#include "LoadMonitor.h"

#include <Preferences.h>
#include <math.h>

#include "SafetyMonitor.h"
#include "MotionController.h"
#include "TempSensors.h"

// ============================================================================
// Preferences Keys (Namespace "rotor")
// ============================================================================
// Baseline-Profile (Kalibrierung)
static const char* KEY_CAL_VALID = "calv";   // bool
static const char* KEY_CAL_CW    = "calcw";  // bytes: LOAD_BINS * uint16
static const char* KEY_CAL_CCW   = "calcc";  // bytes: LOAD_BINS * uint16
static const char* KEY_CAL_TAMB  = "ctA";    // float (Umgebungstemp bei Kalibrierung)
static const char* KEY_CAL_TMOT  = "ctM";    // float (Motortemp bei Kalibrierung)

// Live-Stat (optional persistent erweiterbar)
// Aktuell: nur im RAM (damit keine Flash-Abnutzung entsteht)

// ============================================================================
// Private Helper
// ============================================================================
static float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static uint8_t safeU8Ptr(const uint8_t* p, uint8_t defv) {
  return p ? *p : defv;
}

static float safeFPtr(const float* p, float defv) {
  return p ? *p : defv;
}

static float accIgnoreDeg(const LoadMonitorConfigPointers& cfg) {
  float ig = safeFPtr(cfg.accIgnoreRampDeg, 0.0f);
  if (ig < 0.0f) ig = 0.0f;
  if (ig > 60.0f) ig = 60.0f;
  return ig;
}

// ============================================================================
// LoadMonitor
// ============================================================================

void LoadMonitor::begin(Preferences* prefs,
                        SafetyMonitor* safety,
                        MotionController* motion,
                        TempSensors* temps,
                        const LoadMonitorConfigPointers& cfg) {
  _prefs = prefs;
  _safety = safety;
  _motion = motion;
  _temps = temps;
  _cfg = cfg;

  // Baseline aus Preferences laden
  _calValid = false;
  for (uint8_t i = 0; i < LOAD_BINS; i++) {
    _calCw[i] = 0;
    _calCcw[i] = 0;
    _liveCw[i] = 0;
    _liveCcw[i] = 0;
    _liveCntCw[i] = 0;
    _liveCntCcw[i] = 0;
    _accCw[i] = 0;
    _accCcw[i] = 0;
    _accCntCw[i] = 0;
    _accCntCcw[i] = 0;
  }

  if (_prefs) {
    bool valid = _prefs->getBool(KEY_CAL_VALID, false);
    size_t lenCw = _prefs->getBytesLength(KEY_CAL_CW);
    size_t lenCcw = _prefs->getBytesLength(KEY_CAL_CCW);

    if (valid && lenCw == (size_t)(LOAD_BINS * sizeof(uint16_t)) && lenCcw == (size_t)(LOAD_BINS * sizeof(uint16_t))) {
      _prefs->getBytes(KEY_CAL_CW,  _calCw,  lenCw);
      _prefs->getBytes(KEY_CAL_CCW, _calCcw, lenCcw);
      _calValid = true;
    }
  }

  // Cal-State
  _stage = ST_IDLE;
  _calState = _calValid ? LC_STATE_DONE : LC_STATE_IDLE;
  _calProgress = _calValid ? 100 : 0;

  // Live-State
  _liveMoveActive = false;
  _liveDir = 0;
  _liveStartDeg01 = 0;
  _liveTargetDeg01 = 0;
  for (uint8_t i = 0; i < LOAD_BINS; i++) _binVisited[i] = false;

  _movesUsed = 0;
  _persistDragInc = 0;
  _persistDragDec = 0;
  for (uint8_t i = 0; i < LOAD_BINS; i++) {
    _accCw[i] = 0;
    _accCcw[i] = 0;
    _accCntCw[i] = 0;
    _accCntCcw[i] = 0;
  }

  // Letzte Auswertung
  _lastMeanPct = 0;
  _lastPeakAbsPct = 0;
  _lastCohPct = 0;
  _lastWindDirDeg = 0;
  _lastWindPeakDeg = 0;
  _lastWindPeakPosPct = 0;

  _lastSampleMs = 0;
  _lastAccSampleMs = 0;
}

float LoadMonitor::effectiveIgnoreDeg() const {
  // Wie Etappe2: max(rampDistDeg, calIgnoreRampDeg), Obergrenze 60deg.
  const float ramp = safeFPtr(_cfg.rampDistDeg, 30.0f);
  const float extra = safeFPtr(_cfg.calIgnoreRampDeg, 10.0f);
  float ig = (ramp > extra) ? ramp : extra;
  if (ig < 0.0f) ig = 0.0f;
  if (ig > 60.0f) ig = 60.0f;
  return ig;
}

uint8_t LoadMonitor::calcBinIndex(int32_t deg01) const {
  if (deg01 < 0) deg01 = 0;
  if (deg01 > 36000) deg01 = 36000;

  // 5deg = 500 Deg01
  int32_t idx = deg01 / 500;
  if (idx < 0) idx = 0;
  if (idx > (int32_t)(LOAD_BINS - 1)) idx = LOAD_BINS - 1;
  return (uint8_t)idx;
}

uint16_t LoadMonitor::binCenterDeg(uint8_t idx) const {
  // Center: 2deg in 5deg Bin (0..4 -> 2)
  if (idx >= LOAD_BINS) idx = LOAD_BINS - 1;
  return (uint16_t)(idx * 5 + 2);
}

void LoadMonitor::update(uint32_t nowMs) {
  updateTemperatureWarnings();

  // Gemeinsame Momentaufnahme nur einmal lesen.
  int32_t curDeg01 = 0;
  if (_motion) {
    (void)_motion->getCurrentPositionDeg01(curDeg01);
  }

  uint16_t mvCalOrLive = 0;
  uint16_t mvAcc = 0;
  if (_safety) {
    const SafetyIsSnapshot is = _safety->getIsSnapshot();

    if (_stage == ST_RUN_CW) {
      mvCalOrLive = (uint16_t)clampFloat((float)is.avg1, 0.0f, 65535.0f);
    } else if (_stage == ST_RUN_CCW) {
      mvCalOrLive = (uint16_t)clampFloat((float)is.avg2, 0.0f, 65535.0f);
    } else if (_liveMoveActive) {
      const uint16_t a1 =
          (uint16_t)clampFloat((float)is.avg1, 0.0f, 65535.0f);
      const uint16_t a2 =
          (uint16_t)clampFloat((float)is.avg2, 0.0f, 65535.0f);
      uint16_t mvDir = (_liveDir > 0) ? a1 : a2;
      // Ein-Shunt / asymmetrische Messung: wenn Richtungskanal 0, anderen Kanal nutzen
      if (mvDir == 0 && _liveDir != 0 && (a1 > 0 || a2 > 0)) {
        mvDir = (a1 >= a2) ? a1 : a2;
      }
      mvCalOrLive = mvDir;
      mvAcc = mvDir;
    }
  }

  // Normale Kalibrier-/Live-Statistik bleibt unveraendert langsam.
  const uint32_t sampleEveryMs = 20;
  if ((nowMs - _lastSampleMs) >= sampleEveryMs) {
    _lastSampleMs = nowMs;

    // ----------------------------------------------------------------------
    // Kalibrierfahrt: Samples sammeln
    // ----------------------------------------------------------------------
    if (_stage == ST_RUN_CW || _stage == ST_RUN_CCW) {
      const float igDeg = effectiveIgnoreDeg();
      const int32_t igDeg01 = (int32_t)(igDeg * 100.0f + 0.5f);

      bool inWindow = false;
      if (_stage == ST_RUN_CW) {
        // 0 -> 360
        inWindow = (curDeg01 >= igDeg01) && (curDeg01 <= (36000 - igDeg01));
      } else {
        // 360 -> 0
        inWindow = (curDeg01 <= (36000 - igDeg01)) && (curDeg01 >= igDeg01);
      }

      if (inWindow && mvCalOrLive > 0) {
        uint8_t bin = calcBinIndex(curDeg01);
        calAccumulateSample((_stage == ST_RUN_CW) ? 1 : 2, bin, mvCalOrLive);
      }
    }

    // ----------------------------------------------------------------------
    // Live-Stat: Samples sammeln
    // ----------------------------------------------------------------------
    if (_liveMoveActive) {
      const float minMoveDeg = safeFPtr(_cfg.statMinMoveDeg, 5.0f);
      const int32_t minMoveDeg01 = (int32_t)(minMoveDeg * 100.0f + 0.5f);
      const int32_t moveDistDeg01 = abs(_liveTargetDeg01 - _liveStartDeg01);

      if (moveDistDeg01 >= minMoveDeg01) {
        const float igDeg = effectiveIgnoreDeg();
        const int32_t igDeg01 = (int32_t)(igDeg * 100.0f + 0.5f);

        bool inWindow = false;
        if (_liveDir > 0) {
          int32_t lo = _liveStartDeg01 + igDeg01;
          int32_t hi = _liveTargetDeg01 - igDeg01;
          if (lo <= hi) {
            inWindow = (curDeg01 >= lo) && (curDeg01 <= hi);
          }
        } else if (_liveDir < 0) {
          int32_t hi = _liveStartDeg01 - igDeg01;
          int32_t lo = _liveTargetDeg01 + igDeg01;
          if (lo <= hi) {
            inWindow = (curDeg01 <= hi) && (curDeg01 >= lo);
          }
        }

        if (inWindow && mvCalOrLive > 0) {
          uint8_t bin = calcBinIndex(curDeg01);
          _binVisited[bin] = true;
          liveAccumulateSample((_liveDir > 0) ? 1 : 2, bin, mvCalOrLive);
        }
      }
    }
  }

  // Schnelle Accu-Statistik: dichteres Sampling, eigener Ignore-Wert, keine Kalibrierpflicht.
  const uint32_t accSampleEveryMs = 5;
  if (_liveMoveActive && (nowMs - _lastAccSampleMs) >= accSampleEveryMs) {
    _lastAccSampleMs = nowMs;

    const float minMoveDeg = safeFPtr(_cfg.statMinMoveDeg, 5.0f);
    const int32_t minMoveDeg01 = (int32_t)(minMoveDeg * 100.0f + 0.5f);
    const int32_t moveDistDeg01 = abs(_liveTargetDeg01 - _liveStartDeg01);

    if (moveDistDeg01 >= minMoveDeg01) {
      const float igDeg = accIgnoreDeg(_cfg);
      const int32_t igDeg01 = (int32_t)(igDeg * 100.0f + 0.5f);

      bool inWindow = false;
      if (_liveDir > 0) {
        const int32_t lo = _liveStartDeg01 + igDeg01;
        const int32_t hi = _liveTargetDeg01 - igDeg01;
        if (lo <= hi) {
          inWindow = (curDeg01 >= lo) && (curDeg01 <= hi);
        }
      } else if (_liveDir < 0) {
        const int32_t hi = _liveStartDeg01 - igDeg01;
        const int32_t lo = _liveTargetDeg01 + igDeg01;
        if (lo <= hi) {
          inWindow = (curDeg01 <= hi) && (curDeg01 >= lo);
        }
      }

      if (inWindow && mvAcc > 0) {
        const uint8_t bin = calcBinIndex(curDeg01);
        accAccumulateSample((_liveDir > 0) ? 1 : 2, bin, mvAcc);
      }
    }
  }

  // ------------------------------------------------------------------------
  // Kalibrierfahrt: State Machine
  // ------------------------------------------------------------------------
  if (_calState == LC_STATE_RUNNING) {
    // Safety-Fault -> Abbruch
    if (_safety && _safety->isFault()) {
      abortCalibration(nowMs);
      _calState = LC_STATE_ERROR;
      _calProgress = 0;
      return;
    }

    // Progress berechnen (nur informativ)
    int32_t curDeg01 = 0;
    if (_motion) (void)_motion->getCurrentPositionDeg01(curDeg01);

    if (_stage == ST_RUN_CW) {
      uint32_t p = (uint32_t)((curDeg01 * 50L) / 36000L);
      if (p > 50) p = 50;
      _calProgress = (uint8_t)p;
    } else if (_stage == ST_RUN_CCW) {
      uint32_t p = (uint32_t)(((36000L - curDeg01) * 50L) / 36000L);
      if (p > 50) p = 50;
      _calProgress = (uint8_t)(50 + p);
    } else {
      _calProgress = 0;
    }

    // Stage-Wechsel nur, wenn Motion gerade NICHT aktiv ist
    const bool posActive = _motion ? _motion->isPosActive() : false;

    if (!posActive) {
      if (_stage == ST_MOVE_TO_ZERO) {
        // jetzt CW starten
        if (_motion && _motion->commandSetPosDeg01(36000, nowMs)) {
          _stage = ST_RUN_CW;
        } else {
          _calState = LC_STATE_ERROR;
          _stage = ST_IDLE;
        }
      } else if (_stage == ST_RUN_CW) {
        // jetzt CCW starten
        if (_motion && _motion->commandSetPosDeg01(0, nowMs)) {
          _stage = ST_RUN_CCW;
        } else {
          _calState = LC_STATE_ERROR;
          _stage = ST_IDLE;
        }
      } else if (_stage == ST_RUN_CCW) {
        // Fertig -> speichern
        calFinalizeAndStore();
        _stage = ST_FINISH;
        _calState = LC_STATE_DONE;
        _calProgress = 100;
      }
    }
  }

  // ------------------------------------------------------------------------
  // Live-Stat: Bewegungsende erkennen und bewerten
  // ------------------------------------------------------------------------
  if (_liveMoveActive) {
    const bool posActive = _motion ? _motion->isPosActive() : false;
    if (!posActive) {
      // Bewegung fertig
      liveEvaluateAndWarn();
      _liveMoveActive = false;
      _liveDir = 0;
      for (uint8_t i = 0; i < LOAD_BINS; i++) _binVisited[i] = false;
    }
  }
}

void LoadMonitor::updateTemperatureWarnings() {
  if (!_safety || !_temps) return;

  const float ambWarn = safeFPtr(_cfg.tempWarnAmbientC, 0.0f);
  const float motWarn = safeFPtr(_cfg.tempWarnMotorC, 0.0f);

  const float ambC = _temps->getAmbientC();
  const float motC = _temps->getMotorC();

  // Warnschwelle <=0 = deaktiviert
  if (ambWarn > 0.0f && ambC >= ambWarn) {
    _safety->raiseWarning(SW_TEMP_AMBIENT_HIGH);
  }

  if (motWarn > 0.0f && motC >= motWarn) {
    _safety->raiseWarning(SW_TEMP_MOTOR_HIGH);
  }
}

bool LoadMonitor::startCalibration(uint32_t nowMs) {
  if (!_motion) return false;
  if (_calState == LC_STATE_RUNNING) return false;

  // Joerg: Falls kurz davor noch eine Live-Auswertung aktiv war (z.B. Master hat SETPOSDG gestartet
  // und direkt danach SETCAL gesendet), brechen wir die Live-Auswertung ab.
  // Kalibrierung soll IMMER exklusiv laufen, ohne dass alte Bin-Visited Flags/Live-States stoehren.
  abortLiveMoveTracking();

  // Keine Parallelfahrt
  if (_motion->isPosActive()) return false;

  calResetAccu();

  // Stage 0: erst auf 0 fahren (damit immer gleich gestartet wird)
  if (!_motion->commandSetPosDeg01(0, nowMs)) {
    _calState = LC_STATE_ERROR;
    _stage = ST_IDLE;
    _calProgress = 0;
    return false;
  }

  _calState = LC_STATE_RUNNING;
  _stage = ST_MOVE_TO_ZERO;
  _calProgress = 0;

  return true;
}

void LoadMonitor::abortCalibration(uint32_t nowMs) {
  if (_motion) {
    _motion->commandStopSoft();
  }

  // Kalibrierung abbrechen
  _stage = ST_IDLE;
  _calState = LC_STATE_ABORT;
  _calProgress = 0;

  // Akkus verwerfen
  calResetAccu();
}

bool LoadMonitor::deleteCalibration() {
  if (!_prefs) {
    _calValid = false;
    for (uint8_t i = 0; i < LOAD_BINS; i++) {
      _calCw[i] = 0;
      _calCcw[i] = 0;
    }
    return false;
  }

  _prefs->remove(KEY_CAL_VALID);
  _prefs->remove(KEY_CAL_CW);
  _prefs->remove(KEY_CAL_CCW);
  _prefs->remove(KEY_CAL_TAMB);
  _prefs->remove(KEY_CAL_TMOT);

  _calValid = false;
  for (uint8_t i = 0; i < LOAD_BINS; i++) {
    _calCw[i] = 0;
    _calCcw[i] = 0;
  }
  return true;
}

void LoadMonitor::clearStats() {
  for (uint8_t i = 0; i < LOAD_BINS; i++) {
    _liveCw[i] = 0;
    _liveCcw[i] = 0;
    _liveCntCw[i] = 0;
    _liveCntCcw[i] = 0;
    _accCw[i] = 0;
    _accCcw[i] = 0;
    _accCntCw[i] = 0;
    _accCntCcw[i] = 0;
  }
  _movesUsed = 0;
  _persistDragInc = 0;
  _persistDragDec = 0;

  _lastMeanPct = 0;
  _lastPeakAbsPct = 0;
  _lastCohPct = 0;
  _lastWindDirDeg = 0;
  _lastWindPeakDeg = 0;
  _lastWindPeakPosPct = 0;
}

void LoadMonitor::clearAccStats() {
  for (uint8_t i = 0; i < LOAD_BINS; i++) {
    _accCw[i] = 0;
    _accCcw[i] = 0;
    _accCntCw[i] = 0;
    _accCntCcw[i] = 0;
  }
}

void LoadMonitor::notifyMoveStarted(int32_t startDeg01, int32_t targetDeg01, uint32_t nowMs) {
  (void)nowMs;
  _liveStartDeg01 = startDeg01;
  _liveTargetDeg01 = targetDeg01;

  if (targetDeg01 > startDeg01) _liveDir = 1;
  else if (targetDeg01 < startDeg01) _liveDir = -1;
  else _liveDir = 0;

  _liveMoveActive = (_liveDir != 0);
  for (uint8_t i = 0; i < LOAD_BINS; i++) _binVisited[i] = false;
}

void LoadMonitor::abortLiveMoveTracking() {
  _liveMoveActive = false;
  _liveDir = 0;
  for (uint8_t i = 0; i < LOAD_BINS; i++) {
    _binVisited[i] = false;
  }
}

uint16_t LoadMonitor::getCalBin(uint8_t dir, uint8_t idx) const {
  if (!_calValid) return 0;
  if (idx >= LOAD_BINS) return 0;
  if (dir == 2) return _calCcw[idx];
  return _calCw[idx];
}

uint16_t LoadMonitor::getLiveBin(uint8_t dir, uint8_t idx) const {
  if (idx >= LOAD_BINS) return 0;
  if (dir == 2) return _liveCcw[idx];
  return _liveCw[idx];
}

uint16_t LoadMonitor::getAccBin(uint8_t dir, uint8_t idx) const {
  if (idx >= LOAD_BINS) return 0;
  if (dir == 2) return _accCcw[idx];
  return _accCw[idx];
}

int16_t LoadMonitor::getDeltaPct(uint8_t dir, uint8_t idx) const {
  if (!_calValid) return 0;
  if (idx >= LOAD_BINS) return 0;

  uint16_t base = (dir == 2) ? _calCcw[idx] : _calCw[idx];
  uint16_t live = (dir == 2) ? _liveCcw[idx] : _liveCw[idx];

  if (base == 0 || live == 0) return 0;

  // Prozent: (live-base)/base*100
  int32_t d = (int32_t)live - (int32_t)base;
  int32_t pct = (d * 100) / (int32_t)base;
  if (pct < -32768) pct = -32768;
  if (pct > 32767) pct = 32767;
  return (int16_t)pct;
}

void LoadMonitor::getLoadStat(int16_t& meanPct, int16_t& peakPctAbs, int16_t& cohPct, uint16_t& movesUsed) const {
  meanPct = _lastMeanPct;
  peakPctAbs = _lastPeakAbsPct;
  cohPct = (int16_t)_lastCohPct;
  movesUsed = _movesUsed;
}

void LoadMonitor::getWindInfo(uint16_t& dirDeg, uint16_t& peakDeg, int16_t& peakPctPos, uint16_t& cohPct) const {
  dirDeg = _lastWindDirDeg;
  peakDeg = _lastWindPeakDeg;
  peakPctPos = _lastWindPeakPosPct;
  cohPct = _lastCohPct;
}

void LoadMonitor::calResetAccu() {
  for (uint8_t i = 0; i < LOAD_BINS; i++) {
    _sumCw[i] = 0;
    _sumCcw[i] = 0;
    _cntCw[i] = 0;
    _cntCcw[i] = 0;
  }
}

void LoadMonitor::calAccumulateSample(uint8_t dir, uint8_t bin, uint16_t mv) {
  if (bin >= LOAD_BINS) return;
  if (mv == 0) return;

  if (dir == 2) {
    _sumCcw[bin] += mv;
    if (_cntCcw[bin] < 65000) _cntCcw[bin]++;
  } else {
    _sumCw[bin] += mv;
    if (_cntCw[bin] < 65000) _cntCw[bin]++;
  }
}

void LoadMonitor::calFinalizeAndStore() {
  // Aus Sum/Count -> Mittelwert
  for (uint8_t i = 0; i < LOAD_BINS; i++) {
    if (_cntCw[i] > 0) {
      _calCw[i] = (uint16_t)(_sumCw[i] / (uint32_t)_cntCw[i]);
    } else {
      _calCw[i] = 0;
    }

    if (_cntCcw[i] > 0) {
      _calCcw[i] = (uint16_t)(_sumCcw[i] / (uint32_t)_cntCcw[i]);
    } else {
      _calCcw[i] = 0;
    }
  }

  // Baseline valid
  _calValid = true;

  // Temperaturen bei der Kalibrierung merken (optional fuer spaetere Interpretation)
  float tA = 0.0f;
  float tM = 0.0f;
  if (_temps) {
    tA = _temps->getAmbientC();
    tM = _temps->getMotorC();
  }

  // In Preferences speichern
  if (_prefs) {
    _prefs->putBool(KEY_CAL_VALID, true);
    _prefs->putBytes(KEY_CAL_CW,  _calCw,  LOAD_BINS * sizeof(uint16_t));
    _prefs->putBytes(KEY_CAL_CCW, _calCcw, LOAD_BINS * sizeof(uint16_t));
    _prefs->putFloat(KEY_CAL_TAMB, tA);
    _prefs->putFloat(KEY_CAL_TMOT, tM);
  }

  // Akkus zuruecksetzen
  calResetAccu();
}

void LoadMonitor::liveAccumulateSample(uint8_t dir, uint8_t bin, uint16_t mv) {
  if (bin >= LOAD_BINS) return;
  if (mv == 0) return;

  // Wir wollen eine robuste Statistik ohne Flash-Schreiben.
  // Daher nutzen wir ein kombiniertes Verfahren:
  // - solange count < 200: echter Mittelwert
  // - danach: IIR-Update mit 1/200 (langsam, stabil)

  if (dir == 2) {
    uint16_t& mean = _liveCcw[bin];
    uint16_t& cnt  = _liveCntCcw[bin];

    if (cnt < 200) {
      uint32_t sum = (uint32_t)mean * (uint32_t)cnt + (uint32_t)mv;
      cnt++;
      mean = (uint16_t)(sum / (uint32_t)cnt);
    } else {
      // mean = mean*199/200 + mv*1/200
      uint32_t m = (uint32_t)mean * 199u + (uint32_t)mv;
      mean = (uint16_t)(m / 200u);
    }
  } else {
    uint16_t& mean = _liveCw[bin];
    uint16_t& cnt  = _liveCntCw[bin];

    if (cnt < 200) {
      uint32_t sum = (uint32_t)mean * (uint32_t)cnt + (uint32_t)mv;
      cnt++;
      mean = (uint16_t)(sum / (uint32_t)cnt);
    } else {
      uint32_t m = (uint32_t)mean * 199u + (uint32_t)mv;
      mean = (uint16_t)(m / 200u);
    }
  }
}

void LoadMonitor::accAccumulateSample(uint8_t dir, uint8_t bin, uint16_t mv) {
  if (bin >= LOAD_BINS) return;
  if (mv == 0) return;

  // Schnelle Statistik:
  // - erstes Sample setzt direkt
  // - danach laufen neue Werte mit 50% Gewicht ein
  //   mean = mean*0.5 + mv*0.5
  if (dir == 2) {
    uint16_t& mean = _accCcw[bin];
    uint16_t& cnt = _accCntCcw[bin];
    if (cnt == 0) {
      mean = mv;
      cnt = 1;
      return;
    }
    const uint32_t mixed = (uint32_t)mean * 50u + (uint32_t)mv * 50u + 50u;
    mean = (uint16_t)(mixed / 100u);
    if (cnt < 65535u) cnt++;
  } else {
    uint16_t& mean = _accCw[bin];
    uint16_t& cnt = _accCntCw[bin];
    if (cnt == 0) {
      mean = mv;
      cnt = 1;
      return;
    }
    const uint32_t mixed = (uint32_t)mean * 50u + (uint32_t)mv * 50u + 50u;
    mean = (uint16_t)(mixed / 100u);
    if (cnt < 65535u) cnt++;
  }
}

void LoadMonitor::liveEvaluateAndWarn() {
  // Nur sinnvoll, wenn Baseline existiert
  if (!_calValid) return;

  // Reibung/Wind nur bei Fahrten ab statMinMoveDeg (gleiche Schwelle wie Live/Acc-Bins, Etappe2).
  const float statMinMoveDeg = safeFPtr(_cfg.statMinMoveDeg, 5.0f);
  const int32_t statMinMoveDeg01 = (int32_t)(statMinMoveDeg * 100.0f + 0.5f);
  const int32_t moveDistDeg01 = abs(_liveTargetDeg01 - _liveStartDeg01);
  if (moveDistDeg01 < statMinMoveDeg01) return;

  // Temperatur-Kompensation (kalt -> mehr Reibung ist "ok")
  float tempC = 0.0f;
  if (_temps) tempC = _temps->getAmbientC();

  const float coldT = safeFPtr(_cfg.coldTempDegC, 5.0f);
  const float coldExtra = safeFPtr(_cfg.coldExtraDragPct, 10.0f);

  float dragThr = safeFPtr(_cfg.dragWarnPct, 25.0f);
  if (tempC <= coldT) {
    dragThr += coldExtra;
  }

  const float binsNeedPct = safeFPtr(_cfg.dragWarnBinsPct, 30.0f);
  const uint8_t persistNeed = safeU8Ptr(_cfg.dragPersistMoves, 3);

  const float windPeakThr = safeFPtr(_cfg.windPeakPct, 60.0f);
  const float windCohMin  = safeFPtr(_cfg.windCoherenceMin, 55.0f);

  // Bin-Auswertung nur ueber Bins, die in dieser Fahrt besucht wurden
  int32_t sumPct = 0;
  uint16_t n = 0;

  int16_t peakAbs = 0;
  int16_t peakPos = 0;
  uint8_t peakPosIdx = 0;

  uint16_t aboveThr = 0;
  uint16_t belowThr = 0;

  // Wind-Vektor
  double vx = 0.0;
  double vy = 0.0;
  double sumW = 0.0;

  uint8_t dirSel = (_liveDir > 0) ? 1 : 2;

  for (uint8_t i = 0; i < LOAD_BINS; i++) {
    if (!_binVisited[i]) continue;

    int16_t d = getDeltaPct(dirSel, i);

    // Nur dann zaehlen, wenn Baseline und Live wirklich vorhanden
    uint16_t base = (dirSel == 2) ? _calCcw[i] : _calCw[i];
    uint16_t live = (dirSel == 2) ? _liveCcw[i] : _liveCw[i];
    if (base == 0 || live == 0) continue;

    sumPct += d;
    n++;

    int16_t absd = (d < 0) ? (int16_t)(-d) : d;
    if (absd > peakAbs) peakAbs = absd;

    if (d > peakPos) {
      peakPos = d;
      peakPosIdx = i;
    }

    if (d >= (int16_t)dragThr) aboveThr++;
    if (d <= (int16_t)(-dragThr)) belowThr++;

    // Wind: nur positive Abweichungen als Gewicht
    if (d > 0) {
      const double w = (double)d;
      const double ang = (double)binCenterDeg(i) * (3.14159265358979323846 / 180.0);
      vx += w * cos(ang);
      vy += w * sin(ang);
      sumW += w;
    }
  }

  if (n == 0) return;

  const int16_t meanPct = (int16_t)(sumPct / (int32_t)n);

  // Windrichtung / Kohaerenz
  uint16_t cohPct = 0;
  uint16_t dirDeg = 0;
  if (sumW > 0.01) {
    double mag = sqrt(vx * vx + vy * vy);
    double coh = mag / sumW;
    if (coh < 0.0) coh = 0.0;
    if (coh > 1.0) coh = 1.0;
    cohPct = (uint16_t)(coh * 100.0 + 0.5);

    double ang = atan2(vy, vx) * (180.0 / 3.14159265358979323846);
    if (ang < 0.0) ang += 360.0;
    if (ang >= 360.0) ang -= 360.0;
    dirDeg = (uint16_t)(ang + 0.5);
  }

  // Speichern fuer GETLOADSTAT/GETWIND
  _lastMeanPct = meanPct;
  _lastPeakAbsPct = peakAbs;
  _lastCohPct = cohPct;
  _lastWindDirDeg = dirDeg;
  _lastWindPeakDeg = binCenterDeg(peakPosIdx);
  _lastWindPeakPosPct = peakPos;

  _movesUsed++;

  // Bins-Anteil
  const float abovePct = (n > 0) ? (100.0f * (float)aboveThr / (float)n) : 0.0f;
  const float belowPct = (n > 0) ? (100.0f * (float)belowThr / (float)n) : 0.0f;

  // Mechanische Reibung ZU HOCH (gleichmaessig)
  bool dragUp = (meanPct >= (int16_t)dragThr) && (abovePct >= binsNeedPct);
  if (dragUp) {
    if (_persistDragInc < 250) _persistDragInc++;
  } else {
    if (_persistDragInc > 0) _persistDragInc--; // hysterese
  }

  // Mechanische Reibung ZU NIEDRIG (Antenne fehlt/abgefallen)
  bool dragDown = (meanPct <= (int16_t)(-dragThr)) && (belowPct >= binsNeedPct);
  if (dragDown) {
    if (_persistDragDec < 250) _persistDragDec++;
  } else {
    if (_persistDragDec > 0) _persistDragDec--;
  }

  if (_safety) {
    if (persistNeed > 0) {
      if (_persistDragInc >= persistNeed) {
        _safety->raiseWarning(SW_DRAG_INCREASE);
      }
      if (_persistDragDec >= persistNeed) {
        _safety->raiseWarning(SW_DRAG_DECREASE);
      }
    }

    // Windboe: starker Peak + hohe Kohaerenz
    if ((float)peakPos >= windPeakThr && (float)cohPct >= windCohMin) {
      _safety->raiseWarning(SW_WIND_GUST);
    }
  }
}
