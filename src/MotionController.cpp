#include "MotionController.h"

#include <math.h>

// ============================================================================
// Konstruktor
// ============================================================================

// ============================================================================
// Backlash / Umkehrspiel Hilfsfunktionen (nur ENCTYPE_MOTOR_AXIS)
// ============================================================================

int32_t MotionController::readBacklashDeg01_() const {
  int32_t b = (_cfg.backlashDeg01) ? (*_cfg.backlashDeg01) : 0;
  if (b < 0) b = -b;

  // Nur sinnvoll, wenn Encoder auf Motorachse liegt.
  if (!_encoder || _encoder->getEncoderType() != ENCTYPE_MOTOR_AXIS) {
    return 0;
  }
  return b;
}

int8_t MotionController::engagedDir_() const {
  // _lastMoveDirNonZero wird beim Stop gemerkt und entspricht in der Praxis der "eingeklemmten" Flanke.
  // Wenn wir noch nie gefahren sind, nehmen wir +1 als Default.
  if (_lastMoveDirNonZero != 0) return _lastMoveDirNonZero;
  return +1;
}

int32_t MotionController::encoderDeg01ToOutputDeg01_(int32_t encDeg01) const {
  const int32_t amin = axisMinDeg01();
  const int32_t amax = axisMaxDeg01();

  // Harte Achsgrenzen immer zuerst abfangen.
  // WICHTIGER Randfall fuer MOTOR_AXIS + Backlash:
  // - Nach negativer Bewegung wuerde das einfache Modell OUT = ENC + backlash liefern.
  // - Steht der Encoder aber bereits exakt auf der unteren Achsgrenze (ENC==amin),
  //   dann kann OUT logisch nicht mehr bei z.B. +0,30deg stehen bleiben.
  // - Genau dieser Fall fuehrt sonst dazu, dass eine Sollfahrt auf 0,00deg optisch
  //   bei ~Backlash stehen bleibt (z.B. 0,30deg), obwohl der interne Encoder schon
  //   am Minimum angekommen ist.
  // Darum: an den harten Grenzen hat die Achsgrenze Vorrang vor dem Backlash-Modell.
  if (encDeg01 <= amin) return amin;
  if (encDeg01 >= amax) return amax;

  const int32_t b = readBacklashDeg01_();
  if (b <= 0) {
    return encDeg01;
  }

  // Modell:
  // - Nach positiver Bewegung (engagedDir=+1) gilt: OUT = ENC
  // - Nach negativer Bewegung (engagedDir=-1) gilt: OUT = ENC + backlash
  int32_t outDeg01 = encDeg01;
  if (engagedDir_() < 0) {
    outDeg01 = encDeg01 + b;
  }

  // Ergebnis immer auf den logischen Achsbereich begrenzen.
  if (outDeg01 < amin) outDeg01 = amin;
  if (outDeg01 > amax) outDeg01 = amax;
  return outDeg01;
}

int32_t MotionController::outputDeg01ToEncoderTargetDeg01_(int32_t outDeg01, int8_t desiredDir) const {
  const int32_t amin = axisMinDeg01();
  const int32_t amax = axisMaxDeg01();

  // Ziel selbst immer zuerst auf den logischen Achsbereich begrenzen.
  if (outDeg01 < amin) outDeg01 = amin;
  if (outDeg01 > amax) outDeg01 = amax;

  const int32_t b = readBacklashDeg01_();
  if (b <= 0) return outDeg01;

  // Zielabbildung:
  // - Fahrt in + Richtung: Encoder soll am Ende OUT direkt treffen (keine Korrektur notwendig)
  // - Fahrt in - Richtung: Encoderziel ist OUT - backlash (damit OUT = ENC + backlash wieder stimmt)
  //
  // Randfall an der unteren Achsgrenze:
  // - OUT-Ziel = 0,00deg und negative Fahrt
  // - mathematisch waere Encoderziel = -backlash
  // - das ist im logischen Achsmodell unzulaessig
  //
  // Deshalb an der Grenze sauber auf amin saturieren.
  if (desiredDir < 0) {
    int32_t encTarget = outDeg01 - b;
    if (encTarget < amin) encTarget = amin;
    return encTarget;
  }
  return outDeg01;
}

MotionController::MotionController() {
}

// ============================================================================
// Begin / Reset
// ============================================================================
void MotionController::begin(HalBoard* board, MotorMcpwm* motor, EncoderAxis* encoder, HomingController* homing,
                             const MotionConfigPointers& cfgPtr) {
  _board = board;
  _motor = motor;
  _encoder = encoder;
  _homing = homing;
  _cfg = cfgPtr;

  resetControllerState(millis());
}

void MotionController::resetControllerState(uint32_t nowMs) {
  _posActive = false;
  _targetDeg01 = 0;
  _posStartMs = 0;
  _posTimeoutEvent = false;

  _rampStartDeg01 = 0;
  // Merken, aus welcher Richtung zuletzt wirklich gefahren wurde.
  // Das ist wichtig fuer Umkehrspiel-Kompensation, wenn spaeter aus Stillstand in die Gegenrichtung gestartet wird
  // (nur sinnvoll bei Encoder auf Motorachse).
  if (_moveDir != 0) {
    _lastMoveDirNonZero = _moveDir;
  }
  _moveDir = 0;

  // Debug: zuletzt angewendete Backlash-Kompensation zuruecksetzen.
  _lastBacklashAppliedDeg01 = 0;

  _brakeRequest = false;
  _brakeActive = false;
  _brakeReason = 0;
  _brakeDir = 0;
  _brakeIssuedMs = 0;
  _brakeTargetDeg01 = 0;

_brakeHoldActive = false;
_brakeHoldStartMs = 0;

  _pendingHasTarget = false;
  _pendingTargetDeg01 = 0;
  _pendingSoftStart = false;

  _stopPointActive = false;
  _stopPointDeg01 = 0;
  _stopIssuedMs = 0;

  // Feinphase-Bremse/ Hold abbrechen: neues Ziel kommt -> sofort wieder normale Fahrtlogik
  _motorBrakeRequested = false;
  _fineBrakeHoldActive = false;
  _fineBrakeHoldStartMs = 0;
  _fineLowStopHoldActive = false;
  _fineLowStopHoldStartMs = 0;
  _fineLeadBrakeUsed = false;
  _decelPhaseActive = false;

  _kickActive = false;
  _kickSoftMode = false;
  _kickStartMs = 0;
  _kickStartCounts = 0;
  _kickDir = 0;
  _kickDriveStarted = false;
  _kickDriveStartMs = 0;
  _kickDriveStartCounts = 0;

  _inTol = false;
  _inTolSinceMs = 0;

  _haveLastCounts = false;
  _lastCounts = 0;
  _speedMeasDegPerSec = 0.0f;

  _lastSpeedSampleMs = nowMs;
  _speedCmdDegPerSec = 0.0f;
  _speedCmdRampDegPerSec = 0.0f;

  _iTermPwm = 0.0f;
  _pTermPwm = 0.0f;

  _lastAppliedDuty = 0.0f;

  
  // PWM-Max sofort auf aktuellen Konfig-Wert setzen (keine Anlauf-Rampe durch SETPWM-Glaettung)
  _pwmMaxAbsEffective = (_cfg.pwmMaxAbs) ? clampFloat(*_cfg.pwmMaxAbs, 0.0f, 100.0f) : 100.0f;
_haveMoveBaseline = false;
  _lastMoveCounts = 0;
  _noMoveSinceMs = nowMs;

  // Feinphase (Creep+Bremse) zuruecksetzen
  _motorBrakeRequested = false;
  _fineBrakeHoldActive = false;
  _fineBrakeHoldStartMs = 0;
  _fineLowStopHoldActive = false;
  _fineLowStopHoldStartMs = 0;
  _fineLeadBrakeUsed = false;

  // Feinphase: Exakt-Nachjustage zuruecksetzen
  _fineExactMode = false;
  _fineExactRetryCount = 0;

  // PWM-Max auf aktuellen Konfig-Wert synchronisieren
  _pwmMaxAbsEffective = (_cfg.pwmMaxAbs) ? clampFloat(*_cfg.pwmMaxAbs, 0.0f, 100.0f) : 100.0f;
}

// ============================================================================
// Positionsfahrt-Timeout Event
// ============================================================================
bool MotionController::consumePosTimeoutEvent() {
  const bool had = _posTimeoutEvent;
  _posTimeoutEvent = false;
  return had;
}

// ============================================================================
// Start-Helper: Fahrt wie aus Stillstand starten
// ============================================================================
void MotionController::startKickIfNeeded(uint32_t nowMs, int32_t curDeg01) {
  // Startpunkt fuer Anfahr-Rampe sicher auf die aktuelle Position legen.
  _rampStartDeg01 = curDeg01;

  // Arrival-Flags zuruecksetzen
  _inTol = false;
  _inTolSinceMs = 0;

  // Positions-Timeout neu starten
  _posStartMs = nowMs;

  // Speed-Messung neu initialisieren
  _haveLastCounts = false;
  _speedMeasDegPerSec = 0.0f;
  _lastSpeedSampleMs = nowMs;
  // Startrampe bei 0 beginnen
  _speedCmdDegPerSec = 0.0f;
  _speedCmdRampDegPerSec = 0.0f;

  // PI-Regler zuruecksetzen
  _iTermPwm = 0.0f;
  _pTermPwm = 0.0f;

  // PWM-Ausgang ebenfalls bei 0 starten
  _lastAppliedDuty = 0.0f;

  // Stillstands-Erkennung reset
  _haveMoveBaseline = false;
  _lastMoveCounts = (_encoder) ? _encoder->getCountsRaw() : 0;
  _noMoveSinceMs = nowMs;

  // Optional: KICK aktivieren, bis Encoder-Counts Bewegung zeigen.
  //
  // WICHTIG (Fix fuer Joerg):
  // Bei STOP / Richtungswechsel laufen wir zuerst in eine Bremssequenz und gehen
  // danach wieder in eine neue Positionsfahrt.
  // In genau diesem Uebergang kann _moveDir kurzfristig 0 sein (z.B. wenn wir
  // gerade "auf 0" ausgependelt haben oder Richtung/Speed nicht eindeutig war).
  // Wenn wir KICK dann NICHT aktivieren, bleibt der PWM-Profil-Regler oft bei
  // ~g_minPwm haengen (z.B. 19..20%) und der Motor kommt in der Gegenrichtung
  // nicht sicher los -> danach laeuft die Schleife bis Timeout und/oder SE_STALL.
  //
  // Darum: KICK darf auch dann gestartet werden, wenn _moveDir==0.
  // Die Richtung wird spaeter in der KICK-Logik anhand des Ziel-Fehlers bestimmt.
  const float kickMin = (_cfg.pwmKickMinAbs) ? (*_cfg.pwmKickMinAbs) : 0.0f;
  if (_encoder && kickMin > 0.0f) {
    _kickActive = true;
    _kickStartMs = nowMs;
    _kickStartCounts = _encoder->getCountsRaw();
    // Richtung anhand des Ziel-Fehlers bestimmen (robust auch wenn _moveDir==0).
    const int32_t errDeg01 = computeErrorDeg01(_targetDeg01, curDeg01);
    _kickDir = (errDeg01 > 0) ? +1 : (errDeg01 < 0 ? -1 : 0);

    // KICK-Drive-Phase noch NICHT gestartet (Counts/Jitter ignorieren)
    _kickDriveStarted = false;
    _kickDriveStartMs = 0;
    _kickDriveStartCounts = 0;
  } else {
    _kickActive = false;
    _kickSoftMode = false;
    _kickStartMs = 0;
    _kickStartCounts = 0;
    _kickDir = 0;
    _kickDriveStarted = false;
    _kickDriveStartMs = 0;
    _kickDriveStartCounts = 0;
  }
}

// ============================================================================
// Helper
// ============================================================================
float MotionController::clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

bool MotionController::axisWrapEnabled() const {
  return (_cfg.axisWrapEnabled) ? (*_cfg.axisWrapEnabled) : false;
}

int32_t MotionController::axisMinDeg01() const {
  return (_cfg.axisMinDeg01) ? (*_cfg.axisMinDeg01) : 0;
}

int32_t MotionController::axisMaxDeg01() const {
  return (_cfg.axisMaxDeg01) ? (*_cfg.axisMaxDeg01) : 36000;
}

int32_t MotionController::wrapDeltaDeg01(int32_t curDeg01, int32_t lastDeg01) const {
  // Wrap ueber 0/360deg: kleinste Delta-Bildung
  int32_t d = curDeg01 - lastDeg01;
  if (d > 18000) d -= 36000;
  if (d < -18000) d += 36000;
  return d;
}

int32_t MotionController::computeDeltaDeg01(int32_t curDeg01, int32_t lastDeg01) const {
  if (axisWrapEnabled()) {
    return wrapDeltaDeg01(curDeg01, lastDeg01);
  }
  return (curDeg01 - lastDeg01);
}

int32_t MotionController::computeErrorDeg01(int32_t tgtDeg01, int32_t curDeg01) const {
  // Linearer Fehler mit Achsenbegrenzung (Endschalter-Achse)
  // -> wrap nur, wenn ausdruecklich aktiviert.
  int32_t err = tgtDeg01 - curDeg01;

  if (!axisWrapEnabled()) {
    return err;
  }

  // Wrap-Bereich 0..36000: kleinster Weg
  if (err > 18000) err -= 36000;
  if (err < -18000) err += 36000;
  return err;
}

float MotionController::applyPwmSlew(float targetDuty, float lastDuty, uint32_t dtMs) const {
  float slew = (_cfg.pwmSlewPerSec) ? (*_cfg.pwmSlewPerSec) : 0.0f;

  // Optional: automatische Slew-Rate aus der aktuellen Rampen-Auslegung.
  // Idee: Wenn die Speed-Rampe auf z.B. 3s ausgelegt ist, soll PWM ebenfalls
  // nicht in wenigen Millisekunden auf 100% springen, sondern ueber ~3s ansteigen.
  // Das macht die Rampe am Oszilloskop (und akustisch) sichtbar, ohne die Trajektorie
  // zu "zerstoeren".
  const bool autoSlew = (_cfg.pwmSlewAutoFromRamp && (*_cfg.pwmSlewAutoFromRamp));
  if (slew <= 0.0f && autoSlew) {
    slew = _autoPwmSlewPerSec;
  }

  // Sicherheits-Fallback:
// Wenn weder manuell noch automatisch eine Slew-Rate aktiv ist, kann es
// (z.B. durch SETPWM/SETMAXPWM waehrend der Fahrt) zu harten Spruengen kommen.
// Das ist explizit NICHT gewuenscht. Deshalb erzwingen wir eine konservative
// Default-Slew-Rate.
// Hinweis: Bei aktivem Brems-/Stop-Zustand erlauben wir absichtlich eine
// deutlich groessere Slew-Rate, damit das Abbremsen nicht "weichgespuelt" wird.
if (slew <= 0.0f) {
  slew = 20.0f; // % pro Sekunde (0->100% in ~5s, sehr weich)
}

// Bremsen/Stop soll schnell reagieren (keine lange PWM-Rampe beim Stoppen)
if (_brakeActive || _brakeHoldActive || _motorBrakeRequested) {
  if (slew < 250.0f) slew = 250.0f; // 0->100% in ~0,4s
}


  if (dtMs == 0) return lastDuty;

  const float maxStep = slew * ((float)dtMs / 1000.0f);
  float delta = targetDuty - lastDuty;

  if (delta > maxStep) delta = maxStep;
  if (delta < -maxStep) delta = -maxStep;

  return lastDuty + delta;
}

float MotionController::applyPwmSlewCustom(float targetDuty, float lastDuty, uint32_t dtMs, float slewPerSec) const {
  // Spezielle Slew-Rate, z.B. damit in der Feinzone ein Mindest-PWM schnell genug erreicht wird.
  if (slewPerSec <= 0.0f) return lastDuty;
  if (dtMs == 0) return lastDuty;

  const float maxStep = slewPerSec * ((float)dtMs / 1000.0f);
  float delta = targetDuty - lastDuty;

  if (delta > maxStep) delta = maxStep;
  if (delta < -maxStep) delta = -maxStep;

  return lastDuty + delta;
}

float MotionController::updatePwmMaxAbsEffective(uint32_t dtMs) {
  // Joerg: PWM-Max wird bereits ausserhalb (INO) geglaettet:
  // - g_pwmMaxAbsCmd (Soll) -> g_pwmMaxAbs (Ist)
  // Der MotionController soll hier daher NICHT nochmals eine eigene Rampe bauen,
  // sondern einfach den aktuellen Istwert verwenden.
  (void)dtMs;

  float wanted = (_cfg.pwmMaxAbs) ? (*_cfg.pwmMaxAbs) : 100.0f;
  if (!isfinite(wanted)) wanted = 100.0f;

  // Grenzen 0..100%
  wanted = clampFloat(wanted, 0.0f, 100.0f);
  return wanted;
}


// ============================================================================
// Commands
// ============================================================================
void MotionController::commandStopSoft() {
  resetControllerState(millis());
}

void MotionController::commandStopToCurrentPosition(uint32_t nowMs) {
  // STOP (Joerg):
  // - Position merken, an der STOP ankam (Stop-Punkt)
  // - NICHT abrupt auf 0 gehen!
  // - Stattdessen ein virtuelles Bremsziel in aktueller Bewegungsrichtung erzeugen,
  //   sodass die gleiche Abbremsrampe gefahren wird wie am Ende einer normalen Bewegung.
  // - Nach Stillstand zum Stop-Punkt zurueckfahren (inkl. Feinzone).

  if (!_homing || !_encoder) {
    resetControllerState(nowMs);
    return;
  }

  // Waehrend Homing: STOP ist hier ein harter Abbruch der Motion (Homing steuert selbst)
  if (_homing->isActive()) {
    resetControllerState(nowMs);
    return;
  }

  // Wenn gerade keine Positionsfahrt laeuft: normales Stop.
  if (!_posActive) {
    resetControllerState(nowMs);
    return;
  }

  int32_t curDeg01 = 0;
  _encoder->getPositionDeg01(curDeg01);


  // ------------------------------------------------------------------
  // Joerg: Neues Ziel in Gegenrichtung WAERHREND normaler Abbremsphase
  // ------------------------------------------------------------------
  // Problem:
  // - Wenn wir bereits im Ramp-Down (Abbremsphase) zum aktuellen Ziel sind
  //   und es kommt ein neues Ziel in Gegenrichtung, dann wuerde ein direktes
  //   Umschalten des _targetDeg01 die Abbremsrampe "neu starten".
  // - Dadurch faehrt der Rotor unnötig weit und das Verhalten wirkt traege.
  //
  // Stop-Punkt merken
  _stopPointActive = true;
  _stopPointDeg01 = curDeg01;
  _stopIssuedMs = nowMs;

  // Nach dem Bremsen wollen wir GENAU zu diesem Stop-Punkt zurueck.
  _pendingHasTarget = true;
  _pendingTargetDeg01 = encoderDeg01ToOutputDeg01_(curDeg01);

  // Virtuelles Bremsziel anfordern (Berechnung erfolgt im update(), weil dort accel + Speed verfuegbar sind)
  _brakeRequest = true;
  _brakeActive = false;
  _brakeReason = 1; // STOP
  _brakeIssuedMs = nowMs;

  // Timeout neu starten
  _posStartMs = nowMs;

  // KICK deaktivieren, waehrend wir ausrollen/bremsen
  _kickActive = false;
  // Feinphase-Bremse ggf. abbrechen (STOP hat Vorrang)
  _motorBrakeRequested = false;
  _fineBrakeHoldActive = false;
  _fineBrakeHoldStartMs = 0;
  _fineLeadBrakeUsed = false;
  _fineExactMode = false;
  _fineExactRetryCount = 0;


  // Arrival-Logik zuruecksetzen (sonst koennte er zu frueh "fertig" werden)
  _inTol = false;
  _inTolSinceMs = 0;
}
bool MotionController::getCurrentPositionDeg01(int32_t& outDeg01) const {
  if (!_encoder) return false;
  int32_t encDeg01 = 0;
  _encoder->getPositionDeg01(encDeg01);

  // RS485 gibt immer Abtriebswinkel zurueck.
  outDeg01 = encoderDeg01ToOutputDeg01_(encDeg01);
  return true;
}

void MotionController::commandClearMotionForSetRef(uint32_t nowMs) {
  resetControllerState(nowMs);
}

bool MotionController::commandSetPosDeg01(int32_t tgtDeg01, uint32_t nowMs) {
  if (!_homing || !_homing->isReferenced()) return false;
  if (!_encoder) return false;

  // Start aus Stillstand?
  // (Joerg: Wenn wir am Ziel in der Feinjustage stehen und ein neues Ziel kommt,
  //  soll die Justage sofort abgebrochen werden und die neue Fahrt wie aus Stillstand starten.)
  bool startingFromIdle = (!_posActive);


  // Istposition fuer Richtungsentscheidung
  int32_t curDeg01 = 0;
  _encoder->getPositionDeg01(curDeg01);

  // ------------------------------------------------------------------
  // Joerg: Neues Ziel waehrend Bremsrampe (STOP / Richtungswechsel)
  // ------------------------------------------------------------------
  // Wenn waehrend des Abbremsens (virtuelles Bremsziel / Brems-Hold) schon
  // ein neues Ziel ankommt, darf die PWM NICHT abrupt wieder hochspringen.
  //
  // Wir lassen die laufende Bremsrampe sauber zu Ende laufen und merken uns
  // nur das neue Ziel als "pending".
  // Der Neustart nach dem Brems-Hold erfolgt dann im Soft-KICK-Modus
  // (Slew wie normale Rampe), damit es keinen Ruck gibt.
  if (_posActive && (_brakeRequest || _brakeActive || _brakeHoldActive)) {
    _pendingHasTarget = true;
    _pendingTargetDeg01 = tgtDeg01;
    // Richtungswechsel aus der laufenden Fahrt heraus: normaler Neustart (nicht Soft).
    _pendingSoftStart = true;

    // Ein evtl. gemerkter STOP-Punkt ist ab jetzt irrelevant (neues Ziel hat Vorrang)
    _stopPointActive = false;
    _stopPointDeg01 = 0;
    _stopIssuedMs = 0;

    // Timeout neu starten, damit wir im Bremsen nicht in ein altes Timeout laufen
    _posStartMs = nowMs;
    return true;
  }


  // ------------------------------------------------------------------
  // Joerg: Neues Ziel in Gegenrichtung WAERHREND normaler Abbremsphase (Ramp-Down)
  // ------------------------------------------------------------------
  // Wenn wir bereits im Ramp-Down zum aktuellen Ziel sind und es kommt ein neues Ziel
  // in Gegenrichtung, dann wuerde ein direktes Umschalten des _targetDeg01 die
  // Abbremsrampe "neu starten". Das fuehrt zu unnötig viel Weg und wirkt traege.
  //
  // Gewuenscht:
  // - Die laufende Abbremsrampe zum aktuellen Ziel sauber zu Ende fahren (bis Stillstand)
  // - Danach erst in die Gegenrichtung zum neuen Ziel starten.
  //
  // Umsetzung:
  // - Wenn _decelPhaseActive==true und das neue Ziel die Richtung umkehrt,
  //   merken wir das neue Ziel nur als pending und lassen das aktuelle Ziel unangetastet.
  if (_posActive && !_brakeRequest && !_brakeActive && !_brakeHoldActive && _decelPhaseActive) {
    // Aktuelle Bewegungsrichtung (bevorzugt aus PWM)
    int8_t curDir = 0;
    if (fabsf(_lastAppliedDuty) > 1.0f) {
      curDir = (_lastAppliedDuty > 0.0f) ? +1 : -1;
    } else {
      const int32_t errOld = computeErrorDeg01(_targetDeg01, curDeg01);
      curDir = (errOld > 0) ? +1 : (errOld < 0 ? -1 : 0);
    }

    const int32_t errNew = computeErrorDeg01(tgtDeg01, curDeg01);
    const int8_t newDir = (errNew > 0) ? +1 : (errNew < 0 ? -1 : 0);

    if (curDir != 0 && newDir != 0 && newDir != curDir) {
      // Neues Ziel in Gegenrichtung: erst ausrollen, dann umdrehen
      _pendingHasTarget = true;
      _pendingTargetDeg01 = tgtDeg01;
      _pendingSoftStart = false;

      // Timeout neu starten, damit wir beim Ausrollen nicht in ein altes Timeout laufen
      _posStartMs = nowMs;
      return true;
    }
  }

  // Wenn wir praktisch stehen (Feinjustage/Ankunft), behandeln wir das neue Ziel wie einen Start aus Stillstand.
  // Dadurch gibt es sofort wieder die Standardrampe/KICK, statt in einer langsamen Justage zu verharren.
  if (!startingFromIdle && !_brakeActive && !_brakeRequest) {
    const uint32_t idleHoldMs = 120;
    const bool noMoveStable = (nowMs - _noMoveSinceMs) >= idleHoldMs;

    const float idleMeasTol = 0.4f; // deg/s
    const bool speedSmall = (fabsf(_speedMeasDegPerSec) <= idleMeasTol) && (fabsf(_speedCmdRampDegPerSec) <= 1.0f);

    // Joerg: Sonderfall "Umkehren in der Feinphase".
    // Wenn wir kurz vor dem Ziel sind (Feinfenster) und ein neues Ziel in Gegenrichtung kommt,
    // darf die Logik NICHT in die Bremsfahrt schalten. Grund:
    // - In der Bremsfahrt ist der Fein-Kick deaktiviert und die Brems-PWM wird niemals erhoeht.
    // - Wenn wir dabei am mechanischen Anschlag / hoher Haftreibung stehen, kann das zu einem
    //   scheinbaren "Freeze" fuehren (Motor stoppt, posActive bleibt true) bis zum Timeout.
    // Darum behandeln wir neue Ziele im Feinfenster wie einen Start aus Stillstand:
    // -> Feinjustage/Brems-Hold wird abgebrochen
    // -> neue Fahrt startet sofort mit Standardrampe + KICK.
    int32_t fineWinDeg01 = (_cfg.fineWindowDeg01) ? (*_cfg.fineWindowDeg01) : 0;
    if (fineWinDeg01 < 0) fineWinDeg01 = -fineWinDeg01;
    const int32_t errToOldDeg01 = computeErrorDeg01(_targetDeg01, curDeg01);
    const int32_t absErrToOldDeg01 = (errToOldDeg01 < 0) ? -errToOldDeg01 : errToOldDeg01;
    const bool inFineWindowNow = (fineWinDeg01 > 0 && absErrToOldDeg01 <= fineWinDeg01);

    // Zus. Voraussetzung: wir sind wirklich langsam (sonst wuerde ein harter Richtungswechsel zu ruppig).
    const bool allowFineAbort = speedSmall;


    // WICHTIG:
    // In der Feinphase darf ein neues Ziel in GLEICHER Richtung NICHT wie ein "Start aus Stillstand"
    // behandelt werden, sonst gibt es PWM-Spruenge (Kick/zu schnelle Slew-Rate).
    // Nur bei echter Umkehr (Richtungswechsel) oder wenn wir praktisch stehen, wird "startingFromIdle" gesetzt.
    int8_t curDirHint = _moveDir;
    if (curDirHint == 0) {
      if (fabsf(_lastAppliedDuty) > 1.0f) {
        curDirHint = (_lastAppliedDuty > 0.0f) ? +1 : -1;
      } else {
        curDirHint = (errToOldDeg01 > 0) ? +1 : (errToOldDeg01 < 0 ? -1 : 0);
      }
    }

    const int32_t curOutDeg01_hint = encoderDeg01ToOutputDeg01_(curDeg01);
    const int32_t errNewOut_hint = computeErrorDeg01(tgtDeg01, curOutDeg01_hint);
    const int8_t newDirHint = (errNewOut_hint > 0) ? +1 : (errNewOut_hint < 0 ? -1 : 0);

    const bool fineAbortDueToDirChange =
        (inFineWindowNow && allowFineAbort && curDirHint != 0 && newDirHint != 0 && newDirHint != curDirHint);

    if ((noMoveStable && speedSmall) || fineAbortDueToDirChange) {
      startingFromIdle = true;
    }
  }

  // Wenn wir schon fahren und das Ziel gleich bleibt: nur Timeout neu starten
  if (_posActive && tgtDeg01 == _targetDeg01 && !_brakeActive && !_brakeRequest) {
    _posStartMs = nowMs;
    return true;
  }

  // STOP/Brake abbrechen, wenn ein neues Ziel kommt
  // (Joerg: neues Ziel kann Fahrt verlaengern oder aendern)
  _stopPointActive = false;
  _stopPointDeg01 = 0;
  _stopIssuedMs = 0;

  // Feinphase-Bremse/ Hold abbrechen: neues Ziel kommt -> sofort wieder normale Fahrtlogik
  _motorBrakeRequested = false;
  _fineBrakeHoldActive = false;
  _fineBrakeHoldStartMs = 0;
  _fineLowStopHoldActive = false;
  _fineLowStopHoldStartMs = 0;
  _fineLeadBrakeUsed = false;

  // Zielrichtung bestimmen
  // RS485-Ziel ist Abtrieb (OUT). Der Encoder kann ggf. auf der Motorachse liegen.
  const int32_t curOutDeg01 = encoderDeg01ToOutputDeg01_(curDeg01);
  const int32_t errOutDeg01 = computeErrorDeg01(tgtDeg01, curOutDeg01);
  const int8_t desiredDirNew = (errOutDeg01 > 0) ? +1 : (errOutDeg01 < 0 ? -1 : 0);// Aktuelle Bewegungsrichtung bestimmen
  int8_t curDir = _moveDir;
  if (curDir == 0) {
    if (fabsf(_speedCmdRampDegPerSec) > 1.0f) {
      curDir = (_speedCmdRampDegPerSec > 0.0f) ? +1 : -1;
    } else if (fabsf(_speedMeasDegPerSec) > 1.0f) {
      curDir = (_speedMeasDegPerSec > 0.0f) ? +1 : -1;
    } else {
      curDir = desiredDirNew;
    }
  }

  // Positionsfahrt aktivieren (kein kompletter Reset, damit "Weg verlaengern" glatt bleibt)
  _posActive = true;
  _posStartMs = nowMs;
  _inTol = false;
  _inTolSinceMs = 0;

  if (startingFromIdle) {
    // Startpunkt fuer Anfahr-Rampe setzen
    _rampStartDeg01 = curDeg01;
    _moveDir = desiredDirNew;

    // KICK-Phase aktivieren: PWM wird angehoben, bis Encoder-Counts eine Bewegung zeigen.
    // Normaler Start aus Stillstand: KICK darf hier "hart" sein (keine Soft-Sonderbehandlung).
    _pendingSoftStart = false;
    _kickSoftMode = false;
    _kickActive = true;
    _kickStartMs = nowMs;
    _kickStartCounts = _encoder->getCountsRaw();
    _kickDir = desiredDirNew;

    // Joerg: Counts/Jitter vor dem echten "Ziehen" ignorieren.
    _kickDriveStarted = false;
    _kickDriveStartMs = 0;
    _kickDriveStartCounts = 0;

    // Speed-Messung neu initialisieren
    _haveLastCounts = false;
    _speedMeasDegPerSec = 0.0f;
    _lastSpeedSampleMs = nowMs;
    // Startrampe bei 0 beginnen
    _speedCmdDegPerSec = 0.0f;
    _speedCmdRampDegPerSec = 0.0f;

    // PI-Regler zuruecksetzen
    _iTermPwm = 0.0f;
    _pTermPwm = 0.0f;

    // PWM-Ausgang ebenfalls bei 0 starten
    _lastAppliedDuty = 0.0f;

    // Stillstands-Erkennung reset
    _haveMoveBaseline = false;
    _lastMoveCounts = _encoder->getCountsRaw();
    _noMoveSinceMs = nowMs;

    // ------------------------------------------------------------
    // Umkehrspiel-Kompensation beim Start aus Stillstand
    // ------------------------------------------------------------
    // Wenn der Encoder auf der MOTORACHSE sitzt (ENCTYPE_MOTOR_AXIS), sehen wir das Umkehrspiel nicht direkt.
    // Darum verschieben wir bei einem Richtungswechsel das Ziel um "backlash" in die neue Richtung.
    // Bei ENCTYPE_RING_OUTPUT (Encoder auf Ausgangsachse) wird NICHT kompensiert.
    int32_t finalTgt = outputDeg01ToEncoderTargetDeg01_(tgtDeg01, desiredDirNew);

// Debug: zuletzt angewendete Backlash-Abbildung (Encoderziel - OUT-Ziel)
_lastBacklashAppliedDeg01 = finalTgt - tgtDeg01;

// clamp (wrap ist bei dir AUS)
const int32_t amin = axisMinDeg01();
const int32_t amax = axisMaxDeg01();
if (finalTgt < amin) finalTgt = amin;
if (finalTgt > amax) finalTgt = amax;

// Ziel setzen (interner Regler arbeitet in Encoder-Koordinate)
_targetDeg01 = finalTgt;

    // Brake/Pending loeschen
    _brakeRequest = false;
    _brakeActive = false;
    _brakeReason = 0;
    _pendingHasTarget = false;

_brakeHoldActive = false;
_brakeHoldStartMs = 0;

    return true;
  }

  // Waehrend der Fahrt: gleich/gegen Richtung unterscheiden
  if (desiredDirNew != 0 && curDir != 0 && desiredDirNew != curDir) {
    // Neues Ziel in GEGENRICHTUNG:
    // -> erst virtuelle Bremsfahrt (gleiche Abbremsrampe wie am Bewegungsende)
    // -> nach Stillstand in Gegenrichtung neu starten
    _pendingHasTarget = true;
    _pendingTargetDeg01 = tgtDeg01;

    _brakeRequest = true;
    _brakeActive = false;

_brakeHoldActive = false;
_brakeHoldStartMs = 0;
    _brakeReason = 2; // DIRCHANGE
    _brakeIssuedMs = nowMs;

    // Waehrend Bremsphase kein KICK
    _kickActive = false;

    // Aktuelles Ziel erstmal stehen lassen; update() setzt dann _targetDeg01 = virtuelles Bremsziel
    return true;
  }

  
  // ------------------------------------------------------------------
  // Joerg: Neues Ziel ist naeher als das aktuelle Ziel (gleiche Richtung)
  // ------------------------------------------------------------------
  // Problem:
  // - Wenn waehrend einer laufenden Fahrt ein neues Ziel kommt, das in der gleichen Richtung liegt,
  //   aber deutlich naeher ist (z.B. kurz vor der aktuellen Position), dann wuerde ein direktes
  //   Umschalten des Ziels den Soll-PWM sprunghaft reduzieren -> "hartes Bremsen".
  //
  // Loesung:
  // - Wie beim STOP: erst ueber die definierte Rampenlaenge ausrollen/bremsen (virtuelles Bremsziel),
  //   danach das neue Ziel anfahren (pending).
  if (_posActive && !_brakeRequest && !_brakeActive && !_brakeHoldActive) {
    const int32_t curOutDeg01_tmp = encoderDeg01ToOutputDeg01_(curDeg01);
    const int32_t oldOutDeg01_tmp = encoderDeg01ToOutputDeg01_(_targetDeg01);

    const int32_t errOldOut = computeErrorDeg01(oldOutDeg01_tmp, curOutDeg01_tmp);
    const int32_t errNewOut = computeErrorDeg01(tgtDeg01, curOutDeg01_tmp);

    const int32_t absOldOut = (errOldOut < 0) ? -errOldOut : errOldOut;
    const int32_t absNewOut = (errNewOut < 0) ? -errNewOut : errNewOut;

    const int8_t dirOldOut = (errOldOut > 0) ? +1 : (errOldOut < 0 ? -1 : 0);
    const int8_t dirNewOut = (errNewOut > 0) ? +1 : (errNewOut < 0 ? -1 : 0);

    float rampDistDeg = (_cfg.rampDistDeg) ? (*_cfg.rampDistDeg) : 0.0f;
    if (!isfinite(rampDistDeg) || rampDistDeg < 0.5f) rampDistDeg = 0.5f;
    const int32_t rampDistDeg01 = (int32_t)lroundf(rampDistDeg * 100.0f);

    // Nur wenn das neue Ziel in gleicher Richtung liegt, aber deutlich naeher ist
    // und zudem so nah, dass ein direktes Umschalten zu einem harten Brems-Sprung fuehren wuerde.
    if (dirOldOut != 0 && dirNewOut == dirOldOut && (absNewOut + 2) < absOldOut && absNewOut <= rampDistDeg01) {
      _pendingHasTarget = true;
      _pendingTargetDeg01 = tgtDeg01;
      _pendingSoftStart = true;

      // STOP-Punkt ist hier nicht relevant
      _stopPointActive = false;
      _stopPointDeg01 = 0;
      _stopIssuedMs = 0;

      // Virtuelles Bremsziel anfordern (wie STOP/DIRCHANGE)
      _brakeRequest = true;
      _brakeActive = false;

      _brakeHoldActive = false;
      _brakeHoldStartMs = 0;

      _brakeReason = 3; // RETARGET_CLOSE
      _brakeIssuedMs = nowMs;

      // Waehrend Bremsphase kein KICK
      _kickActive = false;

      // Timeout neu starten
      _posStartMs = nowMs;
      return true;
    }
  }

// Neues Ziel in gleicher Richtung:
  // - wenn wir gerade bremsen wollten: Brake abbrechen
  _brakeRequest = false;
  _brakeActive = false;
  _brakeReason = 0;
  _pendingHasTarget = false;

  _brakeHoldActive = false;
  _brakeHoldStartMs = 0;

  // Wenn wir gerade in der Abbremsphase waren und das neue Ziel weiter weg in gleicher Richtung liegt,
  // starten wir die Beschleunigungsrampe ab "jetzt" neu, damit es keinen PWM-Sprung auf MAX gibt.
  if (_decelPhaseActive && desiredDirNew != 0) {
    const int32_t curOutDeg01_tmp = encoderDeg01ToOutputDeg01_(curDeg01);
    const int32_t oldOutDeg01_tmp = encoderDeg01ToOutputDeg01_(_targetDeg01);

    int32_t absOldOut = computeErrorDeg01(oldOutDeg01_tmp, curOutDeg01_tmp);
    if (absOldOut < 0) absOldOut = -absOldOut;
    int32_t absNewOut = computeErrorDeg01(tgtDeg01, curOutDeg01_tmp);
    if (absNewOut < 0) absNewOut = -absNewOut;

    // Nur wenn das neue Ziel wirklich weiter weg ist (kleine Aenderungen ignorieren)
    if (absNewOut > absOldOut + 2) {
      _rampStartDeg01 = curDeg01;
    }
  }

  int32_t finalTgt = outputDeg01ToEncoderTargetDeg01_(tgtDeg01, desiredDirNew);

  // Debug: zuletzt angewendete Backlash-Abbildung (Encoderziel - OUT-Ziel)
  _lastBacklashAppliedDeg01 = finalTgt - tgtDeg01;

  // clamp (wrap ist bei dir AUS)
  const int32_t amin = axisMinDeg01();
  const int32_t amax = axisMaxDeg01();
  if (finalTgt < amin) finalTgt = amin;
  if (finalTgt > amax) finalTgt = amax;

  _targetDeg01 = finalTgt;

  // Bewegungsrichtung ggf. aktualisieren
  _moveDir = desiredDirNew;
  return true;
}
// ============================================================================
// Update
// ============================================================================
float MotionController::update(uint32_t nowMs, uint32_t dtMs) {
  // Standard: keine aktive Motorbremse angefordert (wird unten ggf. gesetzt)
  _motorBrakeRequested = false;

  // Cruise ist Legacy (wird nicht mehr genutzt)
  if (!_board || !_encoder || !_homing) return 0.0f;

  // Homing laeuft: Motion liefert 0 (Homing steuert Motor direkt)
  if (_homing->isActive()) {
    _lastAppliedDuty = 0.0f;
    return 0.0f;
  }

  // HW Service-Taster (open-loop)
  const float handSpeed = (_cfg.handSpeedPercent) ? (*_cfg.handSpeedPercent) : 25.0f;
  const bool srvLeft  = _board->readServiceLeft();
  const bool srvRight = _board->readServiceRight();

  if (srvLeft && !srvRight) {
    resetControllerState(nowMs);
    _lastAppliedDuty = -handSpeed;
    return -handSpeed;
  }
  if (srvRight && !srvLeft) {
    resetControllerState(nowMs);
    _lastAppliedDuty = +handSpeed;
    return +handSpeed;
  }

  // Keine Positionsfahrt
  if (!_posActive) {
    _lastAppliedDuty = 0.0f;
    return 0.0f;
  }

  // dt absichern (siehe Kommentar in aelteren Versionen)
  if (dtMs == 0) dtMs = 1;
  if (dtMs > 50) dtMs = 50;

  // PWM-Max geglaettet (z.B. bei RS485 SETPWM waehrend der Fahrt)
  const float pwmMaxEff = updatePwmMaxAbsEffective(dtMs);

  // Timeout
  const uint32_t posTimeout = (_cfg.posTimeoutMs) ? (*_cfg.posTimeoutMs) : 60000;
  if ((nowMs - _posStartMs) > posTimeout) {
    // Joerg: Bisher wurde ein Positions-Timeout still beendet (kein Error).
    // Gewuenscht: Timeout wie normaler Fehler behandeln.
    // Die eigentliche Fehlerbehandlung (Safety-Fault + ERR-Broadcast) erfolgt
    // zentral in der INO. Hier setzen wir nur ein Event-Flag.
    resetControllerState(nowMs);
    _posTimeoutEvent = true;
    _lastAppliedDuty = 0.0f;
    return 0.0f;
  }

  // Istposition / Counts
  int32_t curDeg01 = 0;
  _encoder->getPositionDeg01(curDeg01);

  const long countsNow = _encoder->getCountsRaw();
  // Joerg: Fuer Stillstand/KICK benutzen wir bewusst RAW-Counts.
  // Grund: Z-Korrektur / Offset-Korrektur kann die "korrigierten" Counts auch im Stillstand springen lassen
  // (z.B. +/- wenige Counts oder in groesseren Korrekturschritten). Das wuerde _noMoveSinceMs dauernd resetten
  // und STOP/Richtungswechsel in der Brems-Hold-Phase "haengen" lassen.


  // ------------------------------------------------------------
  // Stillstands-Erkennung ueber Encoder-Counts
  // ------------------------------------------------------------
  if (!_haveMoveBaseline) {
    _haveMoveBaseline = true;
    _lastMoveCounts = countsNow;
    _noMoveSinceMs = nowMs;
  } else {
    // Joerg: Bei sehr hochaufloesenden Encodern (z.B. 160k CPR) koennen
    // Einzel-Counts durch Rauschen/Quantisierung flackern. Wenn wir hier schon
    // bei +/-1 Count "Bewegung" erkennen, resetten wir _noMoveSinceMs dauernd.
    // Das verhindert dann sowohl die Ankunfts-Erkennung (Stillstand) als auch
    // die automatische Losbrechhilfe.
    //
    // Darum: "Bewegung" erst ab einem kleinen Count-Schwellwert.
    const long moveDetectCounts = 3; // 3 Counts ~ 0,007deg bei 160k CPR
    long d = countsNow - _lastMoveCounts;
    if (d < 0) d = -d;
    if (d >= moveDetectCounts) {
      _lastMoveCounts = countsNow;
      _noMoveSinceMs = nowMs;
    }
  }

  // ------------------------------------------------------------
  // ------------------------------------------------------------
  // Speed messen (vMeas): nur fuer Debug-Ausgabe (spart CPU).
  // ------------------------------------------------------------
  const bool dbgSpeed = (_cfg.debugEnabled && (*_cfg.debugEnabled));

  if (dbgSpeed) {
    const uint32_t speedIntervalMs = (_cfg.speedMeasIntervalMs) ? (*_cfg.speedMeasIntervalMs) : 25;
    uint32_t dtSpeedMs = nowMs - _lastSpeedSampleMs;

    if (dtSpeedMs >= speedIntervalMs) {
      if (dtSpeedMs == 0) dtSpeedMs = 1;
      _lastSpeedSampleMs = nowMs;
      const float dtSpeed = (float)dtSpeedMs / 1000.0f;

      // Speed-Messung ueber Encoder-Counts (robust)
      const int32_t cpr = _encoder->getCountsPerRevActual();

      if (!_haveLastCounts || cpr <= 0) {
        _haveLastCounts = true;
        _lastCounts = countsNow;
        _speedMeasDegPerSec = 0.0f;
      } else {
        const long dCounts = countsNow - _lastCounts;
        _lastCounts = countsNow;

        const float degPerCount = 360.0f / (float)cpr;
        const float dDeg = (float)dCounts * degPerCount;
        _speedMeasDegPerSec = dDeg / dtSpeed;
      }
    }
  } else {
    // Debug aus: vMeas nicht berechnen. Baselines trotzdem aktualisieren,
    // damit beim spaeteren Einschalten von Debug keine Spruenge entstehen.
    _speedMeasDegPerSec = 0.0f;
    _haveLastCounts = true;
    _lastCounts = countsNow;
    _lastSpeedSampleMs = nowMs;
  }

  // ------------------------------------------------------------
  // Rampenweg bestimmen (primaer Grad, optional Counts)
  // ------------------------------------------------------------
  // Hinweis (Joerg): Im neuen Profil-Modus (PWM-Rampenprofil) nutzen wir KEINE
  // Speed-Trajektorie mehr. Die Rampenlaenge ist eine reine Weggroesse (Grad),
  // ueber die wir PWM von Minimum -> Maximum -> Minimum/Fein-PWM auslaufen lassen.

  // Rampenweg bestimmen (Grad)
  // Hinweis: In den aufgeraeumten Versionen nutzen wir nur noch rampDistDeg.
  // Eine alternative Angabe in Encoder-Counts (rampDistCounts) wurde entfernt,
  // um Stellschrauben und Sonderpfade zu reduzieren.
  float rampDistDeg = (_cfg.rampDistDeg) ? (*_cfg.rampDistDeg) : 0.0f;

  if (rampDistDeg <= 0.0001f) {
    rampDistDeg = 4.0f;
  }
  if (rampDistDeg < 0.05f) rampDistDeg = 0.05f;

  // Auto-Slew aus Speed-Rampe ist hier nicht mehr sinnvoll (wir haben keine
  // Speed-Rampe mehr). Wir lassen applyPwmSlew() auf seinen Default-Fallback
  // (Default-Slew, aktuell 20%/s) zurueckfallen, oder du setzt pwmSlewPerSec explizit.
  _autoPwmSlewPerSec = 0.0f;

  // ------------------------------------------------------------
  // Virtuelles Bremsziel (STOP / Richtungswechsel)
  // ------------------------------------------------------------
  if (_brakeRequest && !_brakeActive) {
    // Bewegungsrichtung bestimmen
    int8_t dir = 0;

    // PWM-Profil-Modus: Richtung bevorzugt aus zuletzt angewendeter PWM ableiten.
    // (Speed-Cmd kann hier 0 sein, weil wir kein Speed-PI mehr nutzen.)
    if (fabsf(_lastAppliedDuty) > 1.0f) {
      dir = (_lastAppliedDuty > 0.0f) ? +1 : -1;
    } else if (fabsf(_speedCmdRampDegPerSec) > 1.0f) {
      dir = (_speedCmdRampDegPerSec > 0.0f) ? +1 : -1;
    } else if (fabsf(_speedMeasDegPerSec) > 1.0f) {
      dir = (_speedMeasDegPerSec > 0.0f) ? +1 : -1;
    } else if (_moveDir != 0) {
      dir = _moveDir;
    }

    // Falls komplett unklar: aus aktuellem Ziel-Fehler ableiten
    if (dir == 0) {
      const int32_t errTmp = computeErrorDeg01(_targetDeg01, curDeg01);
      dir = (errTmp > 0) ? +1 : (errTmp < 0 ? -1 : 0);
    }

    // Bremsweg (PWM-Profil-Modus):
    // Wir nutzen die Rampenlaenge als definierten Auslaufweg.
    // Damit ist STOP/Richtungswechsel deterministisch und braucht keine Speed-Schaetzung.
    float brakeDistDeg = rampDistDeg;

    // Joerg: Wenn waehrend einer laufenden Fahrt ein neues Ziel "sehr nah" kommt (RETARGET_CLOSE),
    // dann kann die volle Rampenlaenge dazu fuehren, dass wir unnötig weit ueberfahren und danach
    // zurueckfahren muessen.
    //
    // Daher: Bremsweg proportional zur aktuell anliegenden PWM (als Proxy fuer aktuelle Geschwindigkeit).
    // - Bei hoher PWM -> voller Bremsweg (wie bisher)
    // - Bei kleiner PWM -> kuerzerer Bremsweg
    //
    // STOP und DIRCHANGE bleiben bewusst bei voller Rampenlaenge.
    if (_brakeReason == 3) { // RETARGET_CLOSE
      const float dutyAbs = fabsf(_lastAppliedDuty);
      float frac = 1.0f;
      if (pwmMaxEff > 1.0f) {
        frac = dutyAbs / pwmMaxEff;
      }
      // Untergrenze, damit es nicht in eine quasi-harte Bremse kippt.
      frac = clampFloat(frac, 0.10f, 1.0f);
      brakeDistDeg *= frac;
    }

    if (!isfinite(brakeDistDeg) || brakeDistDeg < 0.5f) brakeDistDeg = 0.5f;

    const int32_t brakeDistDeg01 = (int32_t)lroundf(brakeDistDeg * 100.0f);

    int32_t brakeTarget = curDeg01 + (int32_t)dir * brakeDistDeg01;

    // Achsenbereich clamp (wrap ist bei dir AUS)
    const int32_t amin = axisMinDeg01();
    const int32_t amax = axisMaxDeg01();
    if (brakeTarget < amin) brakeTarget = amin;
    if (brakeTarget > amax) brakeTarget = amax;

    _brakeTargetDeg01 = brakeTarget;
    _targetDeg01 = brakeTarget;

    _brakeActive = true;
    _brakeRequest = false;
    _brakeDir = dir;

// Brems-Hold zuruecksetzen (wird am Ende der Bremsfahrt aktiviert, um Zittern zu vermeiden)
_brakeHoldActive = false;
_brakeHoldStartMs = 0;


    // Waehrend Bremsfahrt KEIN KICK
    _kickActive = false;

    // Arrival-Erkennung zuruecksetzen (damit Brake-Uebergang sauber ist)
    _inTol = false;
    _inTolSinceMs = 0;

    // Rampe-Startpunkt lassen wir bewusst unveraendert (wir bremsen aus laufender Bewegung)
  }

  const bool brakingNow = _brakeActive;

  // ------------------------------------------------------------
  // Fehler (nach evtl. Bremsziel-Override)
  // ------------------------------------------------------------
  const int32_t errDeg01 = computeErrorDeg01(_targetDeg01, curDeg01);
  const int32_t absErrDeg01 = (errDeg01 < 0) ? -errDeg01 : errDeg01;
  const float absErrDeg = (float)absErrDeg01 / 100.0f;

const float errDeg = (float)errDeg01 / 100.0f;

  // Bewegungsrichtung initialisieren (fuer spaetere Richtungswechsel-Kompensation)
  const int8_t desiredDirToTarget = (errDeg01 > 0) ? +1 : (errDeg01 < 0 ? -1 : 0);
  if (_moveDir == 0 && desiredDirToTarget != 0) {
    _moveDir = desiredDirToTarget;
    _rampStartDeg01 = curDeg01;
  }

  // ------------------------------------------------------------
  // Arrival (Position + Stillstand)
  // ------------------------------------------------------------
  int32_t arriveTolDeg01 = (_cfg.arriveTolDeg01) ? (*_cfg.arriveTolDeg01) : 2;
  if (arriveTolDeg01 < 0) arriveTolDeg01 = -arriveTolDeg01;
  const uint32_t arriveHoldMs = (_cfg.arriveHoldMs) ? (*_cfg.arriveHoldMs) : 200;

  // Joerg: Ziel-Toleranz darf nur auf der "positiven" Seite gelten.
  // Das Fenster ist damit immer [target .. target+tol] und nie unterhalb des Solls.
  // errDeg01 = target - current
  // - errDeg01 <= 0   -> current >= target (wir sind nicht UNTER dem Soll)
  // - errDeg01 >= -tol -> maximal tol ueber Soll (Overshoot erlaubt)
  const bool inPosTol = (errDeg01 <= 0 && errDeg01 >= -arriveTolDeg01);

  if (brakingNow) {
    // Waehrend der Bremsfahrt keine normale Arrival-Logik.
    // Das Umschalten auf Pending/Stop-Punkt passiert im Brems-Hold weiter unten.
    _inTol = false;
    _inTolSinceMs = 0;
  } else {
    // Normale Zielerkennung im PWM-Profil-Modus:
    // Wir nutzen KEINE Speed-Toleranz mehr, sondern reine Stillstands-Erkennung
    // ueber Encoder-Counts (noMoveSinceMs).
    const bool noMoveStable = (nowMs - _noMoveSinceMs) >= arriveHoldMs;

    if (inPosTol && noMoveStable) {
      // Ziel erreicht und stabiler Stillstand.
      // Wenn waehrend der Fahrt schon ein neues Ziel vorgemerkt wurde (pending),
      // starten wir danach automatisch die naechste Positionsfahrt.
      if (_pendingHasTarget) {
        const int32_t nextTgt = _pendingTargetDeg01;
        const bool nextSoft = _pendingSoftStart;

        // pending vor Reset loeschen (Reset wuerde es ohnehin loeschen)
        _pendingHasTarget = false;
        _pendingTargetDeg01 = 0;
        _pendingSoftStart = false;

        resetControllerState(nowMs);
        _lastAppliedDuty = 0.0f;

        // Naechstes Ziel wie aus Stillstand starten. Soft-Start kann bei Bedarf aktiviert werden.
        // (Hier: Standard = normal, aber wir behalten den Schalter fuer spaetere Feintuning.)
        (void)nextSoft; // aktuell ungenutzt (Startlogik entscheidet selbst)
        commandSetPosDeg01(nextTgt, nowMs);
        return 0.0f;
      }

      resetControllerState(nowMs);
      _lastAppliedDuty = 0.0f;
      return 0.0f;
    }

    // Debug/Status: inTol anzeigen, sobald Pos-Tol stimmt
    _inTol = inPosTol;
    if (!inPosTol) {
      _inTolSinceMs = 0;
    }
  }

  // ------------------------------------------------------------
  // Grobphase: Wenn wir "kleben" (Haftreibung / Last) -> KICK erneut aktivieren
  // ------------------------------------------------------------
  // Problem (Joerg): Nach STOP oder Richtungswechsel kann der Motor in der
  // neuen Richtung manchmal nicht mehr anlaufen, weil er nur knapp am Mindest-
  // PWM arbeitet. Dann sieht man im Log z.B.:
  //   v=0.00, m=COARSE, duty~19..20, err>0
  // und es passiert bis zum Timeout nichts.
  //
  // Ursache: KICK kann zu frueh beendet werden (z.B. durch 1-2 Count Flattern),
  // danach liefert das Dreieckprofil nur noch Mindest-PWM.
  //
  // Loesung: Wenn wir in der Grobphase sind und fuer eine gewisse Zeit KEINE
  // eindeutige Encoder-Bewegung sehen, reaktivieren wir KICK automatisch.
  // Dadurch wird PWM wieder schnell angehoben, bis echte Counts kommen.
  if (!brakingNow && !_kickActive) {
    const uint32_t coarseNoMoveKickMs = 250; // nach 250ms ohne Bewegung -> KICK
    const float minErrDeg = 0.30f;           // nur wenn wirklich eine Strecke zu fahren ist

    const uint32_t stuckMs = (nowMs >= _noMoveSinceMs) ? (nowMs - _noMoveSinceMs) : 0;

    // Feinfenster ausnehmen: in der Feinphase gibt es eine eigene Boost-Logik.
    int32_t fineWinCfg = (_cfg.fineWindowDeg01) ? (*_cfg.fineWindowDeg01) : 0;
    if (fineWinCfg < 0) fineWinCfg = -fineWinCfg;
    const bool notInFine = (fineWinCfg <= 0) ? true : (absErrDeg01 > fineWinCfg);

    float kickMinAbsCfg = (_cfg.pwmKickMinAbs) ? (*_cfg.pwmKickMinAbs) : 0.0f;
    if (kickMinAbsCfg < 0.0f) kickMinAbsCfg = -kickMinAbsCfg;

    const float dutyAbs = fabsf(_lastAppliedDuty);

    if (notInFine && kickMinAbsCfg > 0.01f && desiredDirToTarget != 0 && absErrDeg > minErrDeg &&
        stuckMs > coarseNoMoveKickMs && dutyAbs >= (kickMinAbsCfg - 0.5f)) {
      _kickActive = true;
      // Auto-Re-KICK ist immer "hart" (sonst koennte er wieder kleben bleiben).
      _kickSoftMode = false;
      _kickStartMs = nowMs;
      _kickStartCounts = countsNow;
      _kickDir = desiredDirToTarget;

      // Joerg: Counts/Jitter vor dem echten "Ziehen" ignorieren.
      _kickDriveStarted = false;
      _kickDriveStartMs = 0;
      _kickDriveStartCounts = 0;
    }
  }

  // ------------------------------------------------------------
  // KICK-Phase / Losbrechhilfe (nur wenn aktiv)
  // ------------------------------------------------------------
  if (_kickActive) {
    // Wenn kein Fehler mehr da ist, brauchen wir auch keinen KICK
    if (absErrDeg01 == 0) {
      _kickActive = false;
      _kickSoftMode = false;
    } else {
      const long cNow = _encoder->getCountsRaw();
      // Joerg: Sehr kleine Count-Aenderungen koennen bei manchen Encodern auch
      // ohne echte Bewegung auftreten (Rauschen/Flattern). Beim Umschalten
      // nach STOP/Richtungswechsel kann es ausserdem einen kleinen Nachlauf geben.
      //
      // Darum: KICK erst dann "beenden", wenn wir NACH dem echten Anfahrimpuls
      // (KICK-Drive-Phase gestartet) eine eindeutige Bewegung in KICK-Richtung sehen.
      const long detectCounts = 20;
      const uint32_t minDriveMsForDetect = 40;

      bool endKickNow = false;
      if (_kickDriveStarted) {
        const uint32_t driveMs = (nowMs >= _kickDriveStartMs) ? (nowMs - _kickDriveStartMs) : 0;
        if (driveMs >= minDriveMsForDetect) {
          long dCdir = cNow - _kickDriveStartCounts;
          if (_kickDir < 0) dCdir = -dCdir;
          if (dCdir >= detectCounts) {
            endKickNow = true;
          }
        }
      }

      if (endKickNow) {
        // Bewegung erkannt -> Rampe startet ab JETZT
        _kickActive = false;
        _kickSoftMode = false;
        _kickDriveStarted = false;
        _kickDriveStartMs = 0;
        _kickDriveStartCounts = 0;

        _rampStartDeg01 = curDeg01;

        // Speed-Messung und Regler neu initialisieren
        _haveLastCounts = false;
        _speedMeasDegPerSec = 0.0f;
        _lastSpeedSampleMs = nowMs;
        _speedCmdRampDegPerSec = 0.0f;
        _speedCmdDegPerSec = 0.0f;

        _iTermPwm = 0.0f;
        _pTermPwm = 0.0f;
      } else {
        // Noch keine Bewegung -> PWM erhoehen
        float kickMin = (_cfg.pwmKickMinAbs) ? (*_cfg.pwmKickMinAbs) : 0.0f;
        if (kickMin < 0.0f) kickMin = -kickMin;

        int8_t dir = _kickDir;
        if (dir == 0) {
          dir = (errDeg01 > 0) ? +1 : (errDeg01 < 0 ? -1 : 0);
          _kickDir = dir;
        }

        if (dir != 0 && kickMin > 0.01f) {
          const float pwmMax = pwmMaxEff;
          kickMin = clampFloat(kickMin, 0.0f, pwmMax);

          // ------------------------------------------------------------
          // KICK-Target bestimmen
          // ------------------------------------------------------------
          // Normalfall: Target steigt mit der Zeit an, bis Bewegung erkannt wird.
          // Soft-Mode (neues Ziel kam waehrend einer Bremsrampe):
          //   -> wir starten sanfter und halten das Target erstmal bei kickMin,
          //      damit es beim Neustart keinen Ruck gibt.
          float kickTargetAbs = kickMin;
          if (!_kickSoftMode) {
            float kickSlew = _autoPwmSlewPerSec;
            if (!isfinite(kickSlew) || kickSlew <= 0.01f) kickSlew = 200.0f;

            const float t = (float)(nowMs - _kickStartMs) / 1000.0f;
            kickTargetAbs = kickMin + (kickSlew * t);
            kickTargetAbs = clampFloat(kickTargetAbs, kickMin, pwmMax);
          }

          const float kickTarget = (float)dir * kickTargetAbs;

          // ------------------------------------------------------------
          // Slew / Ausgabe
          // ------------------------------------------------------------
          // Normaler KICK: bewusst schnelle Slew (damit er sicher losbricht).
          // Soft-KICK: nutzt normale Slew (wie Rampe), damit beim Neustart nach
          // Bremsrampe keine PWM-Spruenge/"Ruck" entstehen.
          float dutyOut = 0.0f;
          if (_kickSoftMode) {
            dutyOut = applyPwmSlew(kickTarget, _lastAppliedDuty, dtMs);
          } else {
            dutyOut = applyPwmSlewCustom(kickTarget, _lastAppliedDuty, dtMs, 600.0f);
          }
          _lastAppliedDuty = dutyOut;

          // Erst wenn wir wirklich mindestens kickMin "gezogen" haben, starten wir die
          // Bewegungsdetektion. Damit ignorieren wir Count-Jitter/Nachlauf vor dem Impuls.
          if (!_kickDriveStarted && fabsf(dutyOut) >= (kickMin - 0.5f)) {
            _kickDriveStarted = true;
            _kickDriveStartMs = nowMs;
            _kickDriveStartCounts = cNow;
          }

          return dutyOut;
        }
      }
    }
  }

  
// ------------------------------------------------------------
// PWM-Profil (Joerg): Ramp-Up -> Cruise -> Ramp-Down
// ------------------------------------------------------------
// Grobfahrt wird jetzt NICHT mehr ueber Speed-PI gefahren, sondern ueber ein
// einfaches PWM-Profil, positionsgefuehrt ueber Encoder.
//
// Stellschrauben (minimal):
// - pwmMaxAbs        (max PWM)
// - rampDistDeg      (Rampenlaenge in Grad)
// - pwmKickMinAbs    (Mindest-PWM zum sicheren Anlaufen / Haftreibung)
//
// Uebergang zur Feinphase:
// - Die Abbremsrampe laeuft auf finePwmAbs aus (wenn fineWindow aktiv ist).
// - In der Feinphase uebernimmt dann der vorhandene Creep+Bremse-Code.

// Restweg wie bisher: normal |Fehler|, bei Bremsfahrt nur in Bremsrichtung
float remainingDeg = absErrDeg;
if (brakingNow && _brakeDir != 0) {
  float rem = errDeg * (float)_brakeDir;
  if (rem < 0.0f) rem = 0.0f;
  remainingDeg = rem;
}

// Weg seit Rampenstart (fuer Ramp-Up)
const int32_t dStartDeg01 = computeDeltaDeg01(curDeg01, _rampStartDeg01);
float distFromStartDeg = (float)((dStartDeg01 < 0) ? -dStartDeg01 : dStartDeg01) / 100.0f;
if (distFromStartDeg < 0.0f) distFromStartDeg = 0.0f;

// Feinfenster
const int32_t fineWinDeg01 = (_cfg.fineWindowDeg01) ? (*_cfg.fineWindowDeg01) : 0;
float fineWinDeg = 0.0f;
if (fineWinDeg01 > 0) fineWinDeg = (float)fineWinDeg01 / 100.0f;

// Feinphase aktiv? (nur in normaler Fahrt)
bool fineActive = false;
if (!brakingNow) {
  if (fineWinDeg01 > 0 && absErrDeg01 <= fineWinDeg01) {
    fineActive = true;
  }
}

// Ziel-/Toleranzfenster getroffen?
// Joerg: Das Fenster ist jetzt bewusst EINSEITIG.
// Beispiel bei Soll 300,10 und Toleranz 0,02deg:
// - 300,09  => NICHT angekommen
// - 300,10  => angekommen
// - 300,12  => angekommen
// - 300,13  => NICHT mehr im Fenster
// Damit stoppen wir in der Feinphase nur dann hart mit LOW/LOW, wenn der Istwert
// das Soll erreicht oder leicht positiv ueberschritten hat. Unterhalb des Solls
// wird niemals "angekommen" gemeldet.
const bool hitTolWindowNow = (errDeg01 <= 0) && (errDeg01 >= -arriveTolDeg01);

// Einseitige Ankunft (fuer Feinphase-Bremse)
const bool arriveOneSidedNowRaw = hitTolWindowNow;
// Joerg: Einseitiges Ankommen ist IMMER aktiv (Ring und Axis).
// Wir wollen nie "unter" das Soll zurueckkorrigieren, weil sonst bei der 0,1°-Anzeige
// die erste Nachkommastelle kippen kann (z.B. 60,60 -> 60,59).
// Darum ist "ankommen" immer: cur >= tgt und innerhalb der Toleranz (max. +tol).
const bool arriveForFineBrakeNow = arriveOneSidedNowRaw;

// PWM Limits
const float pwmMax = pwmMaxEff;

// Mindest-PWM (Losbrechhilfe) als untere Schranke der Grob-Rampe
float kickMinAbs = (_cfg.pwmKickMinAbs) ? (*_cfg.pwmKickMinAbs) : 0.0f;
if (kickMinAbs < 0.0f) kickMinAbs = -kickMinAbs;
if (kickMinAbs > pwmMax) kickMinAbs = pwmMax;

// Fein-PWM, das die Grob-Rampe am Ende erreichen soll (damit kein Sprung entsteht)
float finePwmAbsForRamp = (_cfg.finePwmAbs) ? (*_cfg.finePwmAbs) : 12.0f;
if (finePwmAbsForRamp < 0.0f) finePwmAbsForRamp = -finePwmAbsForRamp;
if (finePwmAbsForRamp > pwmMax) finePwmAbsForRamp = pwmMax;
if (kickMinAbs > finePwmAbsForRamp) finePwmAbsForRamp = kickMinAbs;

// Ziel-PWM am Ende der Grob-Rampe:
// - wenn Feinfenster aktiv: genau finePwmAbs
// - sonst: 0 (direkt auslaufen/stoppen)
float endAbs = 0.0f;
if (!brakingNow && fineWinDeg01 > 0) {
  endAbs = finePwmAbsForRamp;
}

// Restweg der Grobphase (ohne Feinfenster), damit Ramp-Down exakt auf endAbs auslaeuft
float remainingCoarseDeg = remainingDeg;
if (!brakingNow && fineWinDeg > 0.0f) {
  remainingCoarseDeg = remainingDeg - fineWinDeg;
  if (remainingCoarseDeg < 0.0f) remainingCoarseDeg = 0.0f;
}

// ------------------------------------------------------------
// Brems-Hold am Ende der Bremsfahrt (STOP / Richtungswechsel)
// ------------------------------------------------------------
// Wenn wir in der Bremsfahrt am Ende sind, ziehen wir PWM per Slew auf 0 und
// warten kurz auf Stillstand, bevor wir auf Pending-Ziel/Stop-Punkt umschalten.
if (brakingNow) {
  const uint32_t brakeHoldMs = 80;
  const bool noMoveStable = (nowMs - _noMoveSinceMs) >= brakeHoldMs;

  const bool remGone = (remainingDeg <= 0.0001f);
  const bool dutySmall = (fabsf(_lastAppliedDuty) <= 3.0f);

  // Joerg: WICHTIGER Fix fuer Richtungswechsel/STOP
  // -------------------------------------------------
  // Problem:
  // - Bei STOP oder Richtungswechsel setzen wir ein "virtuelles Bremsziel" (brakeTarget)
  //   und lassen das PWM-Profil auf 0 auslaufen.
  // - In der Praxis kann der Rotor aber schon VOR Erreichen des Bremsziels stehen bleiben
  //   (Haftreibung, Getriebe-Vorspannung, Lastwechsel).
  // - Dann bleibt remainingDeg > 0, das Profil fordert weiter Mindest-PWM (~g_minPwm) an,
  //   aber der Rotor bewegt sich nicht -> Safety erkennt STALL_NO_ENCODER_MOVE (ERR16).
  //
  // Loesung:
  // - Sobald wir in der Bremsfahrt eine stabile Stillstandsphase erkennen (noMoveStable),
  //   schalten wir IN DEN BREMS-HOLD, auch wenn die PWM noch nicht klein ist.
  // - Im Brems-Hold ziehen wir PWM per Slew schnell auf 0 und warten kurz ab.
  // - Danach schalten wir auf Pending-Ziel (Gegenrichtung) bzw. Stop-Punkt um.
  //
  // Ergebnis: Kein "Haengen" am Mindest-PWM im Bremsziel, kein ERR16 beim Umkehren/STOP.

  if (!_brakeHoldActive) {
    // Brems-Hold starten, wenn:
    // - wir das Bremsziel (praktisch) erreicht haben UND PWM schon klein ist
    //   ODER
    // - wir klar stehen (noMoveStable), auch wenn PWM noch nicht klein ist (Fix oben)
    if ((remGone && dutySmall) || noMoveStable) {
      _brakeHoldActive = true;
      _brakeHoldStartMs = nowMs;

      // Reglerzustaende beruhigen
      _iTermPwm = 0.0f;
      _pTermPwm = 0.0f;
      _speedCmdDegPerSec = 0.0f;
      _speedCmdRampDegPerSec = 0.0f;
    }
  }

  if (_brakeHoldActive) {
    // In Hold: kein "Bremsziel treffen" -> sonst pendelt er
    _targetDeg01 = curDeg01;

    float dutyOut = applyPwmSlew(0.0f, _lastAppliedDuty, dtMs);
    _lastAppliedDuty = dutyOut;

    const bool heldLongEnough = (nowMs - _brakeHoldStartMs) >= brakeHoldMs;

    if (noMoveStable && heldLongEnough) {
      _brakeActive = false;
      _brakeReason = 0;
      _brakeHoldActive = false;

      // Arrival-Flags fuer die naechste Fahrt zuruecksetzen
      _inTol = false;
      _inTolSinceMs = 0;

      if (_pendingHasTarget) {
        // _pendingTargetDeg01 ist immer Abtriebswinkel (OUT)
        const int32_t nextOutDeg01 = _pendingTargetDeg01;
        _pendingHasTarget = false;

        // Richtung in OUT-Koordinate bestimmen
        const int32_t curOutDeg01 = encoderDeg01ToOutputDeg01_(curDeg01);
        const int32_t errOut = computeErrorDeg01(nextOutDeg01, curOutDeg01);
        const int8_t nextDirOut = (errOut > 0) ? +1 : (errOut < 0 ? -1 : 0);

        // OUT-Ziel -> Encoder-Ziel abbilden (Backlash)
        int32_t nextTgt = outputDeg01ToEncoderTargetDeg01_(nextOutDeg01, nextDirOut);
        _lastBacklashAppliedDeg01 = nextTgt - nextOutDeg01;

        // clamp (wrap ist bei dir AUS)
        const int32_t amin = axisMinDeg01();
        const int32_t amax = axisMaxDeg01();
        if (nextTgt < amin) nextTgt = amin;
        if (nextTgt > amax) nextTgt = amax;

        // Richtung fuer den Regler aus Encoder-Fehler bestimmen
        const int32_t errEnc = computeErrorDeg01(nextTgt, curDeg01);
        const int8_t nextDir = (errEnc > 0) ? +1 : (errEnc < 0 ? -1 : 0);

        _targetDeg01 = nextTgt;
        _moveDir = nextDir;
        _rampStartDeg01 = curDeg01;

        // Start wie aus Stillstand.
        // Wenn das Ziel WAERHREND der Bremsrampe kam: Soft-KICK, damit kein Ruck entsteht.
        _kickSoftMode = _pendingSoftStart;
        _pendingSoftStart = false;
        startKickIfNeeded(nowMs, curDeg01);
      } else if (_stopPointActive) {
        _targetDeg01 = _stopPointDeg01;

        const int32_t errNext = computeErrorDeg01(_targetDeg01, curDeg01);
        _moveDir = (errNext > 0) ? +1 : (errNext < 0 ? -1 : 0);
        _rampStartDeg01 = curDeg01;

        // STOP-Punkt: normaler Neustart (Soft nur bei Pending-Ziel).
        _kickSoftMode = false;
        _pendingSoftStart = false;
        startKickIfNeeded(nowMs, curDeg01);
      }
    }

    // Waehrend Hold keine Feinphase / keine weitere Logik
    return dutyOut;
  }
}

// ------------------------------------------------------------
// PWM-Rampenprofil berechnen (Grobfahrt)
// ------------------------------------------------------------
float pwmMinAbs = kickMinAbs;
if (pwmMinAbs < 0.0f) pwmMinAbs = 0.0f;
if (pwmMinAbs > pwmMax) pwmMinAbs = pwmMax;

// Ramp-Up (Start -> pwmMax)
// WICHTIG (Joerg): Bei sehr kurzen Fahrten kann es passieren, dass die Start-Rampe
// am Rand des Feinfensters (remainingDeg ~= fineWin) noch NICHT bis zur Fein-PWM
// (endAbs) "hochgezogen" hat. Dann waere das Minimum aus Up/Down < endAbs und beim
// Umschalten auf Feinphase gaebe es einen PWM-Sprung (Ruck).
//
// Loesung: Wir berechnen eine effektive Rampenlaenge fuer Ramp-Up, so dass am Rand des
// Feinfensters mindestens endAbs erreicht wird - ohne zusaetzliche Stellschraube.
//
// totalDistDeg = distFromStart + remaining (gilt im Normalfall, solange wir nicht wrap/brake spielen)
float rampUpDistDegEff = rampDistDeg;
if (!brakingNow && fineWinDeg01 > 0 && endAbs > 0.01f) {
  const float totalDistDeg = distFromStartDeg + remainingDeg;
  float coarseDistDeg = totalDistDeg - fineWinDeg;
  if (coarseDistDeg < 0.0f) coarseDistDeg = 0.0f;

  const float denom = (pwmMax - pwmMinAbs);
  if (denom > 0.001f) {
    // Wie viel Ramp-Up-Anteil brauchen wir, um endAbs zu erreichen?
    float needAlpha = (endAbs - pwmMinAbs) / denom;
    if (needAlpha < 0.0f) needAlpha = 0.0f;
    if (needAlpha > 1.0f) needAlpha = 1.0f;

    // Wenn die verfuegbare Grobstrecke zu kurz ist, verkuerzen wir effektiv die Rampenlaenge.
    // Damit steigt upAlpha schneller und erreicht endAbs rechtzeitig.
    if (needAlpha > 0.0001f) {
      const float distNeededToReachEndAbs = rampDistDeg * needAlpha;
      if (coarseDistDeg < distNeededToReachEndAbs) {
        rampUpDistDegEff = coarseDistDeg / needAlpha;
        if (rampUpDistDegEff < 0.05f) rampUpDistDegEff = 0.05f;
        if (rampUpDistDegEff > rampDistDeg) rampUpDistDegEff = rampDistDeg;
      }
    }
  }
}

float upAlpha = 1.0f;
if (rampUpDistDegEff > 0.001f) {
  upAlpha = distFromStartDeg / rampUpDistDegEff;
}
if (upAlpha < 0.0f) upAlpha = 0.0f;
if (upAlpha > 1.0f) upAlpha = 1.0f;

float pwmUpAbs = pwmMinAbs + (pwmMax - pwmMinAbs) * upAlpha;

// Ramp-Down (pwmMax -> endAbs)
float downAlpha = 1.0f;
if (rampDistDeg > 0.001f) {
  downAlpha = remainingCoarseDeg / rampDistDeg;
}
if (downAlpha < 0.0f) downAlpha = 0.0f;
if (downAlpha > 1.0f) downAlpha = 1.0f;

float pwmDownAbs = endAbs + (pwmMax - endAbs) * downAlpha;

// Joerg: Merker fuer Abbremsphase (Ramp-Down limitiert das Dreieckprofil)
// WICHTIG:
// Ein blosses "pwmDownAbs <= pwmUpAbs" ist zu grob, weil bei Vollfahrt / Plateau
// haeufig beide Kurven auf demselben Wert liegen. Dann waeren wir formal schon in
// der Abbremsphase, obwohl real noch gar nicht abgebremst wird.
// Folge waere bei einem neuen Ziel in gleicher Richtung, aber weiter weg:
// - _decelPhaseActive wird TRUE
// - commandSetPosDeg01() setzt _rampStartDeg01 neu
// - das PWM faellt kurz ab und beschleunigt dann wieder
//
// Gewuenscht ist _decelPhaseActive nur dann, wenn die Ramp-Down-Kurve die Fahrt
// tatsaechlich bereits begrenzt, also wir real im Auslaufbereich sind.
// Darum verlangen wir zusaetzlich:
// - wir sind NICHT am Cruise-Plateau
// - die Rest-Grobfahrstrecke liegt innerhalb der Rampenlaenge
// - Ramp-Down ist mit etwas Reserve kleiner als Ramp-Up
const bool downCurveIsLimiting = (pwmDownAbs < (pwmUpAbs - 0.05f));
const bool insideDecelDistance = (remainingCoarseDeg <= (rampDistDeg + 0.001f));
const bool notCruisingAtTop = (pwmDownAbs < (pwmMax - 0.05f));
_decelPhaseActive = (!brakingNow && downCurveIsLimiting && insideDecelDistance && notCruisingAtTop);


// Dreieckprofil = Minimum aus Up/Down
float desiredAbs = pwmUpAbs;
if (pwmDownAbs < desiredAbs) desiredAbs = pwmDownAbs;

// Bei Bremsfahrt NIEMALS erhoehen (nur auslaufen)
if (brakingNow) {
  const float curAbs = fabsf(_lastAppliedDuty);
  if (curAbs < desiredAbs) desiredAbs = curAbs;
}

// Richtung:
int8_t dir = (errDeg01 > 0) ? +1 : (errDeg01 < 0 ? -1 : 0);
if (brakingNow && _brakeDir != 0) {
  dir = _brakeDir;
}

float uCmd = (dir == 0) ? 0.0f : ((float)dir * desiredAbs);

// Speed-PI ist im PWM-Profil-Modus deaktiviert
_iTermPwm = 0.0f;
_pTermPwm = 0.0f;
_speedCmdDegPerSec = 0.0f;
_speedCmdRampDegPerSec = 0.0f;

  // ------------------------------------------------------------
  // Feinphase (Joerg): Creep-PWM + aktive Bremse (beide Ausgaenge HIGH)
// ------------------------------------------------------------
// In der Feinphase nutzen wir bewusst KEINEN Speed-PI mehr, weil:
// - vMeas bei sehr kleinen Geschwindigkeiten stark quantisiert / unruhig ist
// - Integrator in Zielnaehe gerne "drueckt" und dadurch Ueberschwingen + Rueckfahrt erzeugt
//
// Stattdessen:
// - konstantes, langsames PWM (finePwmAbs) in Richtung des Positionsfehlers
// - sobald Ziel erreicht/ueberschritten (einseitig) ODER optional kurz davor (Lead):
//   Motorbremse anfordern und kurz halten
  if (!brakingNow && fineActive) {
  const float pwmMax = pwmMaxEff;

  float finePwmAbs = (_cfg.finePwmAbs) ? (*_cfg.finePwmAbs) : 12.0f;
  if (finePwmAbs < 0.0f) finePwmAbs = -finePwmAbs;
  if (finePwmAbs > pwmMax) finePwmAbs = pwmMax;

  // Wenn in der Feinphase das PWM zu klein ist, kann der Motor "kleben" (Haftreibung).
  // Darum sorgen wir dafuer, dass finePwmAbs mindestens so gross ist wie die Losbrechhilfe.
  float kickMinAbs = (_cfg.pwmKickMinAbs) ? (*_cfg.pwmKickMinAbs) : 0.0f;
  if (kickMinAbs < 0.0f) kickMinAbs = -kickMinAbs;
  if (kickMinAbs > finePwmAbs) finePwmAbs = kickMinAbs;
  if (finePwmAbs > pwmMax) finePwmAbs = pwmMax;

  const uint32_t fineBrakeHoldMs = (_cfg.fineBrakeHoldMs) ? (*_cfg.fineBrakeHoldMs) : 80;
  int32_t fineBrakeLeadDeg01 = (_cfg.fineBrakeLeadDeg01) ? (*_cfg.fineBrakeLeadDeg01) : 0;
  if (fineBrakeLeadDeg01 < 0) fineBrakeLeadDeg01 = -fineBrakeLeadDeg01;

  // ----------------------------------------------------------
  // LOW-Stop-Hold im Ziel-/Toleranzfenster
  // ----------------------------------------------------------
  // Sobald das Toleranzfenster getroffen wird, schalten wir in der Feinphase
  // nicht weiter mit Creep-PWM nach, sondern geben 0 PWM aus. In der INO fuehrt
  // das zu stopPwm() => beide H-Bruecken LOW.
  //
  // Das ist bewusst etwas "haerter" als das bisherige Anschleichen, weil wir
  // keine zusaetzlichen Mikrokorrekturen mehr erzeugen, solange der Treffer im
  // Fenster bereits gueltig ist.
  if (_fineLowStopHoldActive) {
    _motorBrakeRequested = false;

    if ((nowMs - _fineLowStopHoldStartMs) < fineBrakeHoldMs) {
      _iTermPwm = 0.0f;
      _pTermPwm = 0.0f;
      _speedCmdDegPerSec = 0.0f;
      _speedCmdRampDegPerSec = 0.0f;

      _lastAppliedDuty = 0.0f;
      return 0.0f;
    }

    _fineLowStopHoldActive = false;

    // Nach dem LOW-Hold ist die Bewegung idealerweise bereits ausgerollt.
    // Bleiben wir im Ziel-/Toleranzfenster, ist die Fahrt beendet.
    if (hitTolWindowNow) {
      if (_pendingHasTarget) {
        const int32_t nextTgt = _pendingTargetDeg01;
        const bool nextSoft = _pendingSoftStart;

        _pendingHasTarget = false;
        _pendingTargetDeg01 = 0;
        _pendingSoftStart = false;

        resetControllerState(nowMs);
        _lastAppliedDuty = 0.0f;

        (void)nextSoft;
        commandSetPosDeg01(nextTgt, nowMs);
        return 0.0f;
      }

      resetControllerState(nowMs);
      _lastAppliedDuty = 0.0f;
      return 0.0f;
    }
  }

  // Wenn wir das Ziel-/Toleranzfenster gerade treffen, sofort mit LOW stoppen.
  // Damit verhindern wir, dass die Feinphase noch ueber das Ziel hinausschiebt.
  if (hitTolWindowNow) {
    _fineLowStopHoldActive = true;
    _fineLowStopHoldStartMs = nowMs;
    _motorBrakeRequested = false;

    _iTermPwm = 0.0f;
    _pTermPwm = 0.0f;
    _speedCmdDegPerSec = 0.0f;
    _speedCmdRampDegPerSec = 0.0f;

    _lastAppliedDuty = 0.0f;
    return 0.0f;
  }

  // Wenn wir gerade in einem Brake-Hold sind: Bremse aktiv halten
  if (_fineBrakeHoldActive) {
    if ((nowMs - _fineBrakeHoldStartMs) < fineBrakeHoldMs) {
      _motorBrakeRequested = true;

      // Reglerzustaende beruhigen (wir fahren in der Feinphase open-loop)
      _iTermPwm = 0.0f;
      _pTermPwm = 0.0f;
      _speedCmdDegPerSec = 0.0f;
      _speedCmdRampDegPerSec = 0.0f;

      _lastAppliedDuty = 0.0f;
      return 0.0f;
    } else {
      // Hold fertig
      _fineBrakeHoldActive = false;
      _motorBrakeRequested = false;

      // Joerg: Kein "Exakt-Nachsetzen" in Zielnaehe.
      // Wir duerfen nicht unter das Soll zurueckfahren (sonst kippt die 0,1°-Anzeige
      // und es kommt zum Pendeln). Darum:
      // - Wenn wir einseitig angekommen sind (cur >= tgt und innerhalb der Toleranz): fertig.
      // - Wenn es nur ein Lead-Bremsen vor dem Ziel war: weiter in der Creep-Logik.
      _fineExactMode = false;
      _fineExactRetryCount = 0;
      if (arriveOneSidedNowRaw) {
        // Ziel einseitig erreicht (cur >= tgt innerhalb Toleranz).
        // Wenn waehrend der Fahrt ein neues Ziel vorgemerkt wurde (pending),
        // soll es danach sofort ganz normal (mit Rampen) angefahren werden.
        if (_pendingHasTarget) {
          const int32_t nextTgt = _pendingTargetDeg01;
          const bool nextSoft = _pendingSoftStart;

          // pending vor Reset loeschen (Reset wuerde es ohnehin loeschen)
          _pendingHasTarget = false;
          _pendingTargetDeg01 = 0;
          _pendingSoftStart = false;

          resetControllerState(nowMs);
          _lastAppliedDuty = 0.0f;

          // Naechstes Ziel wie aus Stillstand starten.
          (void)nextSoft; // aktuell ungenutzt
          commandSetPosDeg01(nextTgt, nowMs);
          return 0.0f;
        }

        resetControllerState(nowMs);
        _lastAppliedDuty = 0.0f;
        return 0.0f;
      }

    }
  }

  // Start-Entscheidung: wann bremsen?
  bool startBrake = false;

  // 1) Ziel erreicht? -> bremsen
  if (arriveForFineBrakeNow) {
    startBrake = true;
  }
  // 2) Optional: kurz VOR dem Ziel bremsen (nur im normalen Modus, nicht im Exakt-Modus)
  //
  // errDeg01 = tgt - cur (linear, ohne Wrap):
  // - Fahrt mit _moveDir > 0: Ist liegt noch UNTER dem Ziel -> err > 0 bis zum Treffer.
  //   Lead: 0 < err <= fineBrakeLeadDeg01 (bisheriger Fall).
  // - Fahrt mit _moveDir < 0: Ist liegt noch UEBER dem Ziel -> err < 0 bis zum Treffer.
  //   Lead: -fineBrakeLeadDeg01 <= err < 0 (fehlte bisher -> Inertia schoss mehrere Grad ueber).
  else if (!_fineExactMode && fineBrakeLeadDeg01 > 0 && _moveDir > 0 && errDeg01 > 0 &&
           errDeg01 <= fineBrakeLeadDeg01) {
    if (!_fineLeadBrakeUsed) {
      startBrake = true;
    }
  } else if (!_fineExactMode && fineBrakeLeadDeg01 > 0 && _moveDir < 0 && errDeg01 < 0 &&
             errDeg01 >= -fineBrakeLeadDeg01) {
    if (!_fineLeadBrakeUsed) {
      startBrake = true;
    }
  }

  if (startBrake) {
    _fineBrakeHoldActive = true;
    _fineBrakeHoldStartMs = nowMs;
    _motorBrakeRequested = true;

    // Wenn der Trigger das Lead-Bremsen war: merken, dass wir es schon genutzt haben.
    if (!_fineExactMode && fineBrakeLeadDeg01 > 0 && _moveDir > 0 && errDeg01 > 0 &&
        errDeg01 <= fineBrakeLeadDeg01) {
      _fineLeadBrakeUsed = true;
    } else if (!_fineExactMode && fineBrakeLeadDeg01 > 0 && _moveDir < 0 && errDeg01 < 0 &&
               errDeg01 >= -fineBrakeLeadDeg01) {
      _fineLeadBrakeUsed = true;
    }

    // Reglerzustaende beruhigen
    _iTermPwm = 0.0f;
    _pTermPwm = 0.0f;
    _speedCmdDegPerSec = 0.0f;
    _speedCmdRampDegPerSec = 0.0f;

    _lastAppliedDuty = 0.0f;
    return 0.0f;
  }

  // Creep: konstantes PWM in Richtung des Fehlers
  // Joerg: Wenn der Motor in der Feinphase "klebt" (Haftreibung), reicht ein fixes PWM evtl. nicht aus.
  // Wir nutzen deshalb die vorhandene Stillstands-Erkennung (_noMoveSinceMs) und erhoehen das PWM
  // automatisch, solange keine Encoder-Bewegung erkannt wird.
  //
  // Ohne zusaetzlichen Parameter:
  // - nach kurzer Stillstandszeit rampen wir das PWM nach oben
  // - Obergrenze haengt vom Restfehler ab (nahe am Ziel weniger "Kick" -> weniger Ueberschwingen)
  const uint32_t fineNoMoveKickMs = 120; // nach 120ms ohne Bewegung beginnt die automatische PWM-Erhoehung

  float finePwmAbsEff = finePwmAbs;

  const uint32_t stuckMs = (nowMs >= _noMoveSinceMs) ? (nowMs - _noMoveSinceMs) : 0;

  // Sehr nahe am Ziel darf die automatische Fein-Kick-Erhoehung NICHT mehr eingreifen,
  // sonst schiebt sie den Rotor genau im 0,1deg-Bereich unnötig ueber das Ziel.
  const int32_t fineBoostMinErrDeg01 = (arriveTolDeg01 * 3 > 10) ? (arriveTolDeg01 * 3) : 10; // mindestens 0,10deg

  if (stuckMs > fineNoMoveKickMs && absErrDeg > 0.05f && absErrDeg01 > fineBoostMinErrDeg01) {
    // Obergrenze abhaengig vom Restfehler:
    // - bei grossem Restfehler duerfen wir staerker "kicken"
    // - nahe am Ziel nur wenig Extra
    float extraCap = 5.0f + (5.0f * absErrDeg);  // 0.0deg -> +5%, 2.0deg -> +15%, 4.0deg -> +25%
    extraCap = clampFloat(extraCap, 5.0f, 25.0f);

    float maxFineAbs = finePwmAbs + extraCap;
    if (maxFineAbs > pwmMax) maxFineAbs = pwmMax;

    // PWM-Erhoehung ueber Zeit (Slew), bis Bewegung wieder einsetzt
    const float boostSlewPerSec = 120.0f; // %/s
    const float t = (float)(stuckMs - fineNoMoveKickMs) / 1000.0f;

    finePwmAbsEff = finePwmAbs + (boostSlewPerSec * t);
    finePwmAbsEff = clampFloat(finePwmAbsEff, finePwmAbs, maxFineAbs);
  }

  // Kurze Fahrten (1..2deg) liegen oft komplett im Feinfenster: volles Creep-PWM ist zu viel
  // Schwung, v.a. in Minus-Richtung (mehr Nachlauf, dann Korrektur zurueck).
  // Skalierung nach Restfehler relativ zur Feinzone; Wurzel = nahe am Ziel staerker drosseln.
  if (fineWinDeg01 > 0) {
    float ratio = (float)absErrDeg01 / (float)fineWinDeg01;
    if (ratio > 1.0f) ratio = 1.0f;
    float scale = sqrtf(ratio);
    if (_moveDir < 0) {
      scale *= 0.78f;
    }
    if (scale < 0.25f) scale = 0.25f;
    finePwmAbsEff *= scale;
    const float floorAbs = fmaxf(5.0f, finePwmAbs * 0.22f);
    if (finePwmAbsEff < floorAbs) {
      finePwmAbsEff = floorAbs;
    }
  }

  const int8_t dir = (errDeg01 > 0) ? +1 : (errDeg01 < 0 ? -1 : 0);
  const float uFine = (float)dir * finePwmAbsEff;

  // In der Feinphase schnellere Slew-Rate, damit die Regelung "reaktiv" bleibt.
  // (800%/s => 8% pro 10ms)
  float dutyOutFine = applyPwmSlewCustom(uFine, _lastAppliedDuty, dtMs, 800.0f);
  _lastAppliedDuty = dutyOutFine;

  // Reglerzustaende neutral halten
  _iTermPwm = 0.0f;
  _pTermPwm = 0.0f;
  _speedCmdDegPerSec = 0.0f;
  _speedCmdRampDegPerSec = 0.0f;

  return dutyOutFine;
}


  // PWM-Slew (normal)
  float dutyOut = applyPwmSlew(uCmd, _lastAppliedDuty, dtMs);
  _lastAppliedDuty = dutyOut;

  return dutyOut;

}
// ============================================================================
// Debug Snapshot
// ============================================================================
MotionDebugSnapshot MotionController::getDebugSnapshot(int32_t curDeg01) const {
  MotionDebugSnapshot s;

  s.posActive = _posActive;
  s.curDeg01 = curDeg01;
  s.tgtDeg01 = _targetDeg01;
  s.errDeg01 = computeErrorDeg01(_targetDeg01, curDeg01);
  s.speedMeasDegPerSec = _speedMeasDegPerSec;
  s.speedCmdDegPerSec = _speedCmdRampDegPerSec;

  s.pTermPwm = _pTermPwm;
  s.iTermPwm = _iTermPwm;

  // Backlash/Umkehrspiel-Diagnose
  s.moveDir = _moveDir;
  s.lastMoveDirNonZero = _lastMoveDirNonZero;
  s.backlashCfgDeg01 = (_cfg.backlashDeg01) ? (*_cfg.backlashDeg01) : 0;
  if (s.backlashCfgDeg01 < 0) s.backlashCfgDeg01 = -s.backlashCfgDeg01;
  // Bei Ring-Encoder darf kein Backlash angewendet werden
  if (_encoder && _encoder->getEncoderType() != ENCTYPE_MOTOR_AXIS) {
    s.backlashCfgDeg01 = 0;
  }
  s.backlashAppliedDeg01 = _lastBacklashAppliedDeg01;
  return s;
}
