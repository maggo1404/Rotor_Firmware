#pragma once

#include <Arduino.h>

class Preferences;
class SafetyMonitor;
class MotionController;
class TempSensors;

// Anzahl Bins ueber 0..360deg
static const uint8_t LOAD_BINS = 72; // 360/5 = 72

// Kalibrier-Status (fuer GETCALSTATE)
enum LoadCalPublicState : uint8_t {
  LC_STATE_IDLE    = 0,
  LC_STATE_RUNNING = 1,
  LC_STATE_DONE    = 2,
  LC_STATE_ABORT   = 3,
  LC_STATE_ERROR   = 4,
};

// Pointer-Konfiguration (Werte liegen in der .ino als g_* Variablen)
struct LoadMonitorConfigPointers {
  // Rampenlaenge Positionsfahrt (Grad), wie MotionController (SETRAMP / NVS "ramp").
  float* rampDistDeg = nullptr;

  // Zusaetzliche Ignorier-Rampe (Grad), SETCALIGNDG/cig. Effektiv: max(rampDistDeg, calIgnoreRampDeg).
  float* calIgnoreRampDeg = nullptr;

  // Mindestweg (Grad): Live/Acc-Bins + Auswertung GETLOADSTAT/GETWIND (wie Etappe2).
  float* statMinMoveDeg = nullptr;

  // GETACCBINS: separater Ignore (SETRAPDG/rap), unabhaengig von ramp/cal.
  float* accIgnoreRampDeg = nullptr;

  // Unterhalb coldTempDegC erlauben wir zusaetzliche Reibung (in Prozent)
  float* coldTempDegC = nullptr;
  float* coldExtraDragPct = nullptr;

  // Temperatur-Warnschwellen (Grad C). <=0 deaktiviert.
  float* tempWarnAmbientC = nullptr;
  float* tempWarnMotorC = nullptr;

  // Mechanische Reibung / Getriebe
  float* dragWarnPct = nullptr;        // Schwellwert Mittelwert in Prozent
  float* dragWarnBinsPct = nullptr;    // Wie viele Bins muessen den Schwellwert ueberschreiten (0..100)
  uint8_t* dragPersistMoves = nullptr; // Wie oft hintereinander, bis Warnung gesetzt wird

  // Wind
  float* windPeakPct = nullptr;        // Peak-Schwellwert in Prozent
  float* windCoherenceMin = nullptr;   // Mindest-Kohaerenz in Prozent (0..100)
};

class LoadMonitor {
public:
  LoadMonitor() = default;

  // prefs: Zeiger auf Preferences-Instanz (Namespace "rotor" muss bereits begin() haben)
  void begin(Preferences* prefs,
             SafetyMonitor* safety,
             MotionController* motion,
             TempSensors* temps,
             const LoadMonitorConfigPointers& cfg);

  // Muss in loop() regelmaessig aufgerufen werden
  void update(uint32_t nowMs);

  // ----------------------------------------------------------
  // Kalibrierung
  // ----------------------------------------------------------
  // Startet die Kalibrierfahrt (0 -> 360 -> 0).
  // Voraussetzungen:
  // - Achse muss referenziert sein (wird im Dispatcher geprueft)
  // - Es darf keine andere Positionsfahrt laufen
  bool startCalibration(uint32_t nowMs);

  // Abbruch (z.B. ABORTCAL oder Safety-Fault)
  void abortCalibration(uint32_t nowMs);

  // Loescht gespeicherte Baseline (Preferences)
  bool deleteCalibration();

  // Loescht Live-Statistik (Preferences bleiben unveraendert)
  void clearStats();

  // Loescht nur die schnelle Accu-Statistik fuer GETACCBINS.
  void clearAccStats();

  // Status fuer GETCALSTATE
  uint8_t getCalState() const { return _calState; }
  uint8_t getCalProgress() const { return _calProgress; }

  // True, solange die Kalibrierfahrt laeuft (wichtig fuer Deadman/Keepalive)
  bool isCalibrationRunning() const { return _calState == LC_STATE_RUNNING; }

  // Baseline vorhanden?
  bool hasCalibration() const { return _calValid; }

  // ----------------------------------------------------------
  // Live-Statistik: Hook fuer SETPOSDG
  // ----------------------------------------------------------
  void notifyMoveStarted(int32_t startDeg01, int32_t targetDeg01, uint32_t nowMs);

  // Abbruch der aktuellen Live-Auswertung (z.B. STOP/NSTOP oder Safety-Fault)
  void abortLiveMoveTracking();

  // ----------------------------------------------------------
  // Abfragen (fuer RS485)
  // ----------------------------------------------------------
  // dir: 1=CW/positiv (IS1), 2=CCW/negativ (IS2)
  uint16_t getCalBin(uint8_t dir, uint8_t idx) const;
  uint16_t getLiveBin(uint8_t dir, uint8_t idx) const;
  uint16_t getAccBin(uint8_t dir, uint8_t idx) const;
  int16_t  getDeltaPct(uint8_t dir, uint8_t idx) const;

  // Kompakte Auswertung
  // meanPct: mittlere Abweichung in Prozent (ueber die zuletzt ausgewertete grosse Fahrt)
  // peakPctAbs: groesster Betrag einer Bin-Abweichung (letzte Auswertung)
  // cohPct: Kohaerenz (Windindikator) 0..100
  // movesUsed: wie viele grosse Fahrten in die Live-Statistik eingeflossen sind
  void getLoadStat(int16_t& meanPct, int16_t& peakPctAbs, int16_t& cohPct, uint16_t& movesUsed) const;

  // Wind-Info
  // dirDeg: resultierende Windrichtung (0..359)
  // peakDeg: Winkel der staerksten Bin
  // peakPctPos: staerkster positiver Peak (in Prozent)
  // cohPct: Kohaerenz 0..100
  void getWindInfo(uint16_t& dirDeg, uint16_t& peakDeg, int16_t& peakPctPos, uint16_t& cohPct) const;

private:
  // interne Cal-Stages
  enum CalStage : uint8_t {
    ST_IDLE = 0,
    ST_MOVE_TO_ZERO,
    ST_RUN_CW,
    ST_RUN_CCW,
    ST_FINISH,
  };

  uint8_t calcBinIndex(int32_t deg01) const;
  uint16_t binCenterDeg(uint8_t idx) const;

  float effectiveIgnoreDeg() const;

  void updateTemperatureWarnings();

  void calResetAccu();
  void calAccumulateSample(uint8_t dir, uint8_t bin, uint16_t mv);
  void calFinalizeAndStore();

  void liveAccumulateSample(uint8_t dir, uint8_t bin, uint16_t mv);
  void accAccumulateSample(uint8_t dir, uint8_t bin, uint16_t mv);
  void liveEvaluateAndWarn();

  void computeLastMetrics();

private:
  Preferences* _prefs = nullptr;
  SafetyMonitor* _safety = nullptr;
  MotionController* _motion = nullptr;
  TempSensors* _temps = nullptr;
  LoadMonitorConfigPointers _cfg{};

  // Baseline (Kalibrierung)
  bool _calValid = false;
  uint16_t _calCw[LOAD_BINS] = {0};
  uint16_t _calCcw[LOAD_BINS] = {0};

  // Live-Stat (geglaettet)
  uint16_t _liveCw[LOAD_BINS] = {0};
  uint16_t _liveCcw[LOAD_BINS] = {0};
  uint16_t _liveCntCw[LOAD_BINS] = {0};
  uint16_t _liveCntCcw[LOAD_BINS] = {0};

  // Schnelle Accu-Statistik fuer kurzfristige Lastveraenderungen.
  // - getrennt nach Richtung
  // - reagiert deutlich schneller als _live*
  // - funktioniert auch ohne Kalibrierung
  uint16_t _accCw[LOAD_BINS] = {0};
  uint16_t _accCcw[LOAD_BINS] = {0};
  uint16_t _accCntCw[LOAD_BINS] = {0};
  uint16_t _accCntCcw[LOAD_BINS] = {0};

  uint16_t _movesUsed = 0;

  // Kalibrier-Akkumulator
  uint32_t _sumCw[LOAD_BINS] = {0};
  uint32_t _sumCcw[LOAD_BINS] = {0};
  uint16_t _cntCw[LOAD_BINS] = {0};
  uint16_t _cntCcw[LOAD_BINS] = {0};

  // Kalibrier-Status
  uint8_t _calState = LC_STATE_IDLE;
  uint8_t _calProgress = 0;
  CalStage _stage = ST_IDLE;

  // Kalibrier-Bewegungsparameter
  int32_t _calStartDeg01 = 0;
  int32_t _calTargetDeg01 = 0;
  int8_t  _calDir = 0;

  // Live-Move
  bool _liveMoveActive = false;
  int32_t _liveStartDeg01 = 0;
  int32_t _liveTargetDeg01 = 0;
  int8_t  _liveDir = 0;
  bool _binVisited[LOAD_BINS] = {false};

  // Persistenz-Zaehler fuer Warnungen
  uint8_t _persistDragInc = 0;
  uint8_t _persistDragDec = 0;

  // Letzte berechnete Metriken (fuer GETLOADSTAT/GETWIND)
  int16_t _lastMeanPct = 0;
  int16_t _lastPeakAbsPct = 0;
  uint16_t _lastCohPct = 0;
  uint16_t _lastWindDirDeg = 0;
  uint16_t _lastWindPeakDeg = 0;
  int16_t  _lastWindPeakPosPct = 0;

  // Sampling-Raster
  uint32_t _lastSampleMs = 0;
  uint32_t _lastAccSampleMs = 0;
};
