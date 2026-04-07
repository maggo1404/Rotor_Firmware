#pragma once

#include <Arduino.h>

#include "HalBoard.h"
#include "MotorMcpwm.h"
#include "EncoderAxis.h"
#include "HomingController.h"

// ============================================================================
// MotionConfigPointers: Pointer auf INO-Globals (spaeter EEPROM/NVS)
// ============================================================================
// Alle Parameter sind Pointer, damit du sie spaeter leicht auf EEPROM/NVS umstellen kannst.
// Wenn ein Pointer nullptr ist, wird im MotionController ein sinnvoller Default genutzt.
// ============================================================================
struct MotionConfigPointers {
  // Bedienung
  // Wird aktuell nur fuer "Hand/Jog"-Befehle genutzt (z.B. RS485: HANDSPEED).
  const float* handSpeedPercent = nullptr;

  // Timeouts
  const uint32_t* posTimeoutMs = nullptr;

  // Arrival (Position + Stillstand)
  const int32_t*  arriveTolDeg01 = nullptr;            // z.B. 2 => 0,02deg
  const uint32_t* arriveHoldMs   = nullptr;            // z.B. 200ms

  // Speed-Messung: Mindest-Intervall
  const uint32_t* speedMeasIntervalMs = nullptr;       // z.B. 25

  // Debug-Schalter (nur fuer Debug-Rechenaufwand, z.B. vMeas).
  // Wenn nullptr oder false: vMeas wird nicht berechnet (spart CPU, weniger Quantisierung bei wenig Counts).
  const bool* debugEnabled = nullptr;

  // Trajektorie / Rampen
  // Rampenlaenge in Grad (Beschleunigen UND Abbremsen nutzen die gleiche Distanz)
  // Beispiel: 30.0f => nach 30deg (wenn genug Strecke vorhanden) soll PWM-Max erreicht werden.
  const float*    rampDistDeg         = nullptr;

  // Feinphase (Zielnaehe): einfache Endphase
  const int32_t*  fineWindowDeg01     = nullptr;       // 0,01deg (Restdistanz)
  const float*    finePwmAbs          = nullptr;       // % PWM (Betrag)
  const uint32_t* fineBrakeHoldMs     = nullptr;       // ms
  const int32_t*  fineBrakeLeadDeg01  = nullptr;       // 0,01deg (positiv) vor dem Ziel

  // PWM Limits / Slew
  const float*    pwmMaxAbs         = nullptr;         // max |PWM%|
  const float*    pwmKickMinAbs     = nullptr;         // Minimal-PWM zum Losbrechen (KICK), 0 => aus
  const float*    pwmSlewPerSec     = nullptr;         // optional, 0 => aus
  const bool*     pwmSlewAutoFromRamp = nullptr;       // true => PWM-Slew automatisch aus Rampendistanz ableiten

  // Umkehrspiel (Backlash) in 0,01deg: wird beim Richtungswechsel als Ziel-Offset beruecksichtigt
  const int32_t* backlashDeg01 = nullptr;

  // Achsenbereich / Wrap (Endschalter-Achse => wrap AUS!)
  const bool*    axisWrapEnabled = nullptr;            // false = linearer Fehler
  const int32_t* axisMinDeg01    = nullptr;            // z.B. 0
  const int32_t* axisMaxDeg01    = nullptr;            // z.B. 36000
};


// ============================================================================
// Debug Snapshot
// ============================================================================
struct MotionDebugSnapshot {
  bool posActive = false;
  int32_t curDeg01 = 0;
  int32_t tgtDeg01 = 0;
  int32_t errDeg01 = 0;
  float speedMeasDegPerSec = 0.0f;
  float speedCmdDegPerSec  = 0.0f;

  float pTermPwm = 0.0f;
  float iTermPwm = 0.0f;

  // ------------------------------------------------------------
  // Backlash/Umkehrspiel-Diagnose
  // ------------------------------------------------------------
  // Aktuelle Bewegungsrichtung (+1/-1/0) gemaess MotionController.
  int8_t moveDir = 0;
  // Letzte bekannte Bewegungsrichtung ungleich 0 (bleibt auch im Stillstand erhalten).
  int8_t lastMoveDirNonZero = 0;
  // Konfiguriertes Umkehrspiel (0,01deg). Wird bei ENCTYPE_RING_OUTPUT intern ignoriert.
  int32_t backlashCfgDeg01 = 0;
  // Zuletzt tatsaechlich als Ziel-Offset angewendetes Umkehrspiel (0,01deg).
  // 0 => aktuell keine Kompensation aktiv.
  int32_t backlashAppliedDeg01 = 0;
};

// ============================================================================
// MotionController
// ============================================================================
class MotionController {
public:
  MotionController();

  void begin(HalBoard* board, MotorMcpwm* motor, EncoderAxis* encoder, HomingController* homing,
             const MotionConfigPointers& cfgPtr);

  // Haupt-Update: liefert gewuenschtes DutySigned (OHNE Safety!)
  float update(uint32_t nowMs, uint32_t dtMs);

  void commandStopSoft();
  void commandStopToCurrentPosition(uint32_t nowMs);
  void commandClearMotionForSetRef(uint32_t nowMs);
  bool commandSetPosDeg01(int32_t tgtDeg01, uint32_t nowMs);

  bool isPosActive() const { return _posActive; }

  // ------------------------------------------------------------
  // Positionsfahrt-Timeout Event
  // ------------------------------------------------------------
  // Wenn die Positionsfahrt laenger als posTimeoutMs dauert, beendet der
  // MotionController die Fahrt (Duty=0) und setzt ein Event-Flag.
  //
  // Hintergrund:
  // - Bisher wurde die Fahrt still beendet (kein Fehler). Du wolltest aber,
  //   dass ein Timeout wie ein normaler Fehler behandelt wird.
  // - Das eigentliche Fehler-Latching und der RS485 ERR-Broadcast passieren
  //   zentral ueber SafetyMonitor in der INO.
  //
  // Diese Funktion liefert true GENAU EINMAL pro Timeout-Ereignis.
  bool consumePosTimeoutEvent();

  // Joerg: true, wenn wir gerade eine Bremssequenz (STOP / Richtungswechsel) fahren.
  // In dieser Phase darf Safety NICHT ueber Encoder-Stall ausloesen.
  bool isBrakeSequenceActive() const { return _brakeRequest || _brakeActive || _brakeHoldActive; }

  // Wird in der Feinphase angefordert: aktive Motorbremse (beide Ausgaenge HIGH).
  // Die eigentliche Umsetzung erfolgt in der INO, damit Safety-Faults immer Vorrang haben.
  bool isMotorBrakeRequested() const { return _motorBrakeRequested; }

  // Feinphase: Exakt-Treffer-Nachjustage (1x) wenn wir innerhalb der
  // einseitigen Toleranz landen aber nicht exakt auf dem Ziel stehen.
  bool isFineExactMode() const { return _fineExactMode; }
  uint8_t getFineExactRetryCount() const { return _fineExactRetryCount; }
  // Aktuelle Position (fuer GETPOSDG / STOP-Punkt)
  bool getCurrentPositionDeg01(int32_t& outDeg01) const;

  MotionDebugSnapshot getDebugSnapshot(int32_t curDeg01) const;

private:

  // --------------------------------------------------------------------------
  // Backlash / Umkehrspiel bei Encoder auf Motorachse
  // - RS485-Positionen sind immer Abtriebswinkel (Ausgangsachse).
  // - Encoder auf Motorachse misst "Motorwinkel in Abtriebsgrad umgerechnet".
  // - Je nach letzter Bewegungsrichtung ist ein Offset (Backlash) aktiv.
  // --------------------------------------------------------------------------
  int32_t readBacklashDeg01_() const;
  int8_t  engagedDir_() const; // +1 = positive Flanke, -1 = negative Flanke, 0 = unbekannt
  int32_t encoderDeg01ToOutputDeg01_(int32_t encDeg01) const;
  int32_t outputDeg01ToEncoderTargetDeg01_(int32_t outDeg01, int8_t desiredDir) const;

  static float clampFloat(float v, float lo, float hi);

  // Achsbereich
  bool axisWrapEnabled() const;
  int32_t axisMinDeg01() const;
  int32_t axisMaxDeg01() const;

  // Fehlerberechnung: linear (Endschalter) oder wrap (nur wenn gewollt)
  int32_t computeErrorDeg01(int32_t tgtDeg01, int32_t curDeg01) const;

  // Delta fuer Speed-Messung: ebenfalls wrap nur wenn wirklich erlaubt
  int32_t computeDeltaDeg01(int32_t curDeg01, int32_t lastDeg01) const;
  int32_t wrapDeltaDeg01(int32_t curDeg01, int32_t lastDeg01) const;

  // PWM-Slew (optional)
  float applyPwmSlew(float targetDuty, float lastDuty, uint32_t dtMs) const;
  // PWM-Slew mit explizit vorgegebener Slew-Rate (z.B. fuer kurze "Losbrechhilfe" in der Feinzone).
  // slewPerSec: % pro Sekunde
  float applyPwmSlewCustom(float targetDuty, float lastDuty, uint32_t dtMs, float slewPerSec) const;
// PWM-Max (z.B. via RS485 SETPWM) soll waehrend der Fahrt nicht sprunghaft aendern.
  // Darum glaetten wir den effektiven PWM-Max intern mit einer Slew-Rate.
  float updatePwmMaxAbsEffective(uint32_t dtMs);


  void resetControllerState(uint32_t nowMs);

  // Start einer Positionsfahrt wie aus Stillstand.
  // Setzt Mess-/Reglerzustaende zurueck und aktiviert optional die KICK-Phase,
  // damit der Motor sicher loslaeuft (statische Reibung), bis Encoder-Counts Bewegung zeigen.
  void startKickIfNeeded(uint32_t nowMs, int32_t curDeg01);

private:
  HalBoard* _board = nullptr;
  MotorMcpwm* _motor = nullptr;
  EncoderAxis* _encoder = nullptr;
  HomingController* _homing = nullptr;

  MotionConfigPointers _cfg{};

  // Position
  bool _posActive = false;
  int32_t _targetDeg01 = 0;
  uint32_t _posStartMs = 0;
  // Wird gesetzt, wenn posTimeoutMs ueberschritten wurde (siehe consumePosTimeoutEvent()).
  bool _posTimeoutEvent = false;
  // Startpunkt fuer Anfahr-Rampe (in 0,01deg)
  // Wird gesetzt, wenn eine Positionsfahrt aus dem Stillstand startet oder wenn wir die Richtung wechseln.
  int32_t _rampStartDeg01 = 0;
  // Bewegungsrichtung, zu der _rampStartDeg01 gehoert.
  // +1 = positive Richtung, -1 = negative Richtung, 0 = unbekannt/steht.
  int8_t _moveDir = 0;

  // Letzte Bewegungsrichtung ungleich 0.
  // Wird genutzt, um bei ENCTYPE_MOTOR_AXIS auch nach einem Stillstand zu wissen,
  // aus welcher Richtung "vorgeladen" wurde (Umkehrspiel-Kompensation beim erneuten Anfahren).
  int8_t _lastMoveDirNonZero = 0;

  // Zuletzt angewendeter Backlash-Offset (0,01deg). Nur zur Diagnose/Debug.
  int32_t _lastBacklashAppliedDeg01 = 0;

  // Brems- / Umkehr-Sequenz (Joerg):
  // STOP und Ziel in Gegenrichtung sollen NICHT abrupt stoppen, sondern die gleiche
  // Abbremsrampe nutzen wie am Ende einer normalen Bewegung.
  // Dafuer wird ein virtuelles Bremsziel in aktueller Bewegungsrichtung erzeugt.
  bool _brakeRequest = false;      // wurde ein Brake angefordert (STOP oder Dirchange)?
  bool _brakeActive  = false;      // fahren wir gerade auf das virtuelle Bremsziel?
  uint8_t _brakeReason = 0;        // 1=STOP, 2=DIRCHANGE
  int8_t _brakeDir = 0;            // Richtung der Bremsfahrt (+1/-1)
  uint32_t _brakeIssuedMs = 0;     // Zeitpunkt der Anforderung
  int32_t _brakeTargetDeg01 = 0;   // virtuelles Bremsziel

// Bremsfahrt-Hold: Wenn wir am Ende der Bremsfahrt nahe 0 sind, gehen wir in einen
// reinen Stillstands-Hold (PWM -> 0) um jegliches "Zittern" / Vor-Zurueck zu vermeiden.
bool _brakeHoldActive = false;
uint32_t _brakeHoldStartMs = 0;


  // Pending-Ziel nach dem Bremsen (Stop-Punkt oder neues Gegenrichtungs-Ziel)
  bool _pendingHasTarget = false;
  int32_t _pendingTargetDeg01 = 0;

  // Joerg:
  // Wenn ein neues Ziel WAERHREND einer Bremssequenz (STOP / Richtungswechsel)
  // ankommt, wollen wir danach WEICH neu starten.
  // Hintergrund:
  // - Der Rotor ist in diesem Moment noch am Ausrollen / Abbremsen.
  // - Ein "harter" KICK mit sehr hoher Slew-Rate kann dann beim Neustart
  //   zu einem deutlichen Ruck fuehren.
  // - Darum merken wir uns diesen Sonderfall und verwenden nach dem Brems-Hold
  //   einen weichen KICK (Slew wie normale Rampe).
  bool _pendingSoftStart = false;

  // STOP-Punkt: damit STOP nie abrupt stoppt und nach Ausrollen sicher zurueckfaehrt
  bool _stopPointActive = false;
  int32_t _stopPointDeg01 = 0;
  uint32_t _stopIssuedMs = 0;

  // KICK/Anfahrhilfe: Wenn der Motor am Start noch nicht laeuft (statische Reibung),
  // erhoehen wir PWM, bis Encoder-Counts eine Bewegung zeigen. Danach uebernimmt die normale Rampe.
  bool _kickActive = false;
  // KICK im "Soft-Mode": Duty wird nur mit normaler Slew-Rate angehoben.
  // Wird verwendet, wenn ein neues Ziel waehrend der Bremsrampe kam.
  bool _kickSoftMode = false;
  uint32_t _kickStartMs = 0;
  long _kickStartCounts = 0;
  int8_t _kickDir = 0;

  // Joerg: KICK muss robust gegen "Count-Flattern" / Nachlauf sein.
  // Beim Umschalten nach STOP/Richtungswechsel kann der Encoder noch wenige
  // Counts liefern, obwohl der neue KICK noch gar nicht wirklich "zieht".
  // Wenn wir den KICK dann zu frueh beenden, bleibt die Grobrampe bei
  // ~g_minPwm haengen und der Motor kommt in Gegenrichtung nicht sicher los.
  bool _kickDriveStarted = false;
  uint32_t _kickDriveStartMs = 0;
  long _kickDriveStartCounts = 0;

  // Arrival
  bool _inTol = false;
  uint32_t _inTolSinceMs = 0;

  // Speed Messung
  bool _haveLastCounts = false;
  long _lastCounts = 0;
  float _speedMeasDegPerSec = 0.0f;

  uint32_t _lastSpeedSampleMs = 0;
  // Speed Command
  float _speedCmdDegPerSec = 0.0f;
  float _speedCmdRampDegPerSec = 0.0f;

  // PI Regler
  float _iTermPwm = 0.0f;
  float _pTermPwm = 0.0f;

  // PWM
  float _lastAppliedDuty = 0.0f;
  float _autoPwmSlewPerSec = 0.0f;

  // Geglaetteter PWM-Max (%). Verhindert harte Spruenge, wenn z.B. per RS485 SETPWM waehrend der Fahrt
  // die maximale PWM geaendert wird.
  float _pwmMaxAbsEffective = NAN;

  // Stillstands-/Losbrech-Erkennung:
  // Wir merken uns, ob sich Encoder-Counts bewegen. Wenn wir bei sehr kleiner
  // Sollgeschwindigkeit stehen bleiben (z.B. nach Richtungswechsel nahe 0),
  // starten wir automatisch wieder die KICK-Phase.
  bool _haveMoveBaseline = false;
  long _lastMoveCounts = 0;
  uint32_t _noMoveSinceMs = 0;

  // Feinphase: _motorBrakeRequested bleibt false (keine aktive Zwischenbremse mehr);
  // wird in der INO ausgewertet, falls spaeter wieder genutzt.
  bool _motorBrakeRequested = false;

  // Legacy-Merker (frueher Lead-/Fenbremse und LOW-Zwischenhalt); werden bei Reset/SetPos geloescht.
  bool _fineBrakeHoldActive = false;
  uint32_t _fineBrakeHoldStartMs = 0;

  bool _fineLowStopHoldActive = false;
  uint32_t _fineLowStopHoldStartMs = 0;

  bool _fineLeadBrakeUsed = false;

  // ------------------------------------------------------------
  // Grobprofil-Phase: Merker, ob wir gerade im Ramp-Down (Abbremsphase) sind
  // ------------------------------------------------------------
  // Dieser Merker wird im update() aus dem PWM-Dreieckprofil abgeleitet.
  // Er wird genutzt, um bei einem neuen Ziel in Gegenrichtung (SETPOSDG)
  // waehrend der Abbremsphase NICHT die Rampe neu zu starten, sondern erst
  // sauber zum Stillstand zu kommen und danach umzudrehen.
  bool _decelPhaseActive = false;

  // ------------------------------------------------------------
  // Feinphase: Exakt-Nachjustage (Joerg)
  // ------------------------------------------------------------
  // In einer Richtung kann es vorkommen, dass wir nach der einseitigen
  // Ankunft (innerhalb der Toleranz, z.B. -1/-2 Deg01) nicht exakt auf 0
  // stehen bleiben. In diesem Fall machen wir genau EINEN zusaetzlichen
  // "Treffer-Versuch":
  // - _fineExactMode=true  => in der Feinphase wird "ankommen" nur bei err==0
  // - _fineExactRetryCount => damit wir nicht in eine Endlosschleife geraten
  bool _fineExactMode = false;
  uint8_t _fineExactRetryCount = 0;

  // Debug

  // Timeout-Event (wird von der INO abgeholt, um SE_POS_TIMEOUT zu latchen)
  
};
