#include "Rs485Dispatcher.h"

#include <math.h>
#include <Preferences.h>

// Vorwaertsdeklarationen (werden in dieser Datei spaeter definiert)
static int32_t computeChecksumScaled100Local(uint8_t src, uint8_t dst, const String& params);
static String  formatScaled100Local(int32_t scaled);

#include "HalBoard.h"
#include "SafetyMonitor.h"
#include "HomingController.h"
#include "MotionController.h"
#include "LoadMonitor.h"
#include "TempSensors.h"


// ============================================================================
// Kleine interne Helper (Pointer-sicher)
// ============================================================================

// uint8_t sicher lesen
static uint8_t safeU8(const uint8_t* p, uint8_t fallback) {
  return p ? *p : fallback;
}

// bool sicher lesen
static bool safeBool(const bool* p, bool fallback) {
  return p ? *p : fallback;
}

// uint32_t sicher lesen
static uint32_t safeU32(const uint32_t* p, uint32_t fallback) {
  return p ? *p : fallback;
}

// int32_t sicher lesen
static int32_t safeI32(const int32_t* p, int32_t fallback) {
  return p ? *p : fallback;
}

// ============================================================================
// Public
// ============================================================================

void Rs485Dispatcher::begin(Rs485Proto* proto,
                           HalBoard* board,
                           SafetyMonitor* safety,
                           HomingController* homing,
                           MotionController* motion,
                           LoadMonitor* loadMon,
                           TempSensors* temps,
                           const Rs485DispatcherConfig& cfg) {
  // Referenzen auf die Module merken
  _rs485 = proto;
  _board = board;
  _safety = safety;
  _homing = homing;
  _motion = motion;
  _loadMon = loadMon;
  _temps = temps;

  // Konfig (Pointer auf globale Parameter) merken
  _cfg = cfg;
}

void Rs485Dispatcher::update(uint32_t nowMs) {
  // Holt alle bereits komplett empfangenen Frames aus der RX-Queue
  // und verarbeitet sie.
  // Der eigentliche UART-Empfang laeuft jetzt in einem eigenen FreeRTOS-Task
  // innerhalb von Rs485Proto und ist damit nicht mehr direkt vom loop()-Takt abhaengig.
  if (!_rs485) return;

  Rs485Frame f;
  while (_rs485->poll(f)) {
    onFrame(f, nowMs);
  }
}

void Rs485Dispatcher::updateHomingKickRetry(uint32_t nowMs) {
  // Optionaler "Kick": falls Homing in einem undefinierten Zustand haengt,
  // wird es nach einer Wartezeit erneut gestartet.
  //
  // Verhalten ist absichtlich so gehalten wie im alten .ino:
  // - nur aktiv solange "nicht referenziert"
  // - wenn Homing aktiv ODER referenziert -> tries = 0
  // - wenn Zeit erreicht -> tries--, next = now + spacing, homing.start()

  if (!(_cfg.homingKickTries && _cfg.homingKickNextMs && _cfg.homingKickMaxTries && _cfg.homingKickSpacingMs)) return;
  if (!_homing) return;

  if (*_cfg.homingKickTries == 0) return;

  // Sobald referenziert -> Kick aus
  if (_homing->isReferenced()) {
    *_cfg.homingKickTries = 0;
    return;
  }

  // Wenn Homing aktiv -> Kick aus (wir wollen nicht waehrenddessen neu starten)
  if (_homing->isActive()) {
    *_cfg.homingKickTries = 0;
    return;
  }

  // Wartezeit noch nicht erreicht
  if (nowMs < *_cfg.homingKickNextMs) return;

  // Kick ausfuehren
  if (*_cfg.homingKickTries > 0) {
    (*_cfg.homingKickTries)--;
  }

  uint32_t spacing = safeU32(_cfg.homingKickSpacingMs, 150);
  *_cfg.homingKickNextMs = nowMs + spacing;

  if (safeBool(_cfg.debug, false)) {
    Serial.print("[HOMING] Kick-Retry: start() triesLeft=");
    Serial.println((int)*_cfg.homingKickTries);
  }

  _homing->start();
}

void Rs485Dispatcher::onFrame(const Rs485Frame& f, uint32_t nowMs) {
  // 1) Logging (auch bei invalid)
  logRs485FrameToSerial(f);

  // 2) Ungueltige Frames: Wenn sie an uns adressiert sind, geben wir ein NAK zurueck,
  //    damit man eine falsche Checksumme sofort erkennt (sonst wuerde das Frame still
  //    verworfen und der Master sieht "kein ACK").
  if (!f.valid) {
    const uint8_t own = safeU8(_cfg.ownSlaveId, 0);
    if (f.cmd.length() > 0 && f.slave == own) {
      const int32_t expScaled = computeChecksumScaled100Local(f.master, f.slave, f.params);
      const String expStr = formatScaled100Local(expScaled);
      // Format: NAK_<CMD>:BADCHK:<expected>
      sendNak(f.master, f.cmd, String("BADCHK:") + expStr);
    }
    return;
  }

  // 3) Dispatch
  handleCommand(f, nowMs);
}

void Rs485Dispatcher::sendErrBroadcast(uint8_t errCode) {
  if (!_rs485) return;

  // In deinem bisherigen Protokoll geht ERR an Slave=0 (Master-Adresse).
  // Dadurch bekommt der Master zuverlaessig den Fehler, ohne Broadcast-Kollisionen.
  const uint8_t ownId = safeU8(_cfg.ownSlaveId, 0);
  _rs485->sendFrame(ownId, 0, "ERR", String(errCode));
}

// ============================================================================
// Private: Senden
// ============================================================================

void Rs485Dispatcher::sendAck(uint8_t master, const String& cmd, const String& params) {
  if (!_rs485) return;
  const uint8_t ownId = safeU8(_cfg.ownSlaveId, 0);
  _rs485->sendFrame(ownId, master, "ACK_" + cmd, params);
}

void Rs485Dispatcher::sendNak(uint8_t master, const String& cmd, const String& params) {
  if (!_rs485) return;
  const uint8_t ownId = safeU8(_cfg.ownSlaveId, 0);
  _rs485->sendFrame(ownId, master, "NAK_" + cmd, params);
}

// ============================================================================
// Private: Debug / Logging
// ============================================================================

void Rs485Dispatcher::logRs485FrameToSerial(const Rs485Frame& f) {
  // RS485-Frame-Logging soll komplett am Debug haengen.
  if (!(safeBool(_cfg.debug, false) && safeBool(_cfg.logFrames, false))) return;

  Serial.print("[RS485][RX] valid=");
  Serial.print(f.valid ? "1" : "0");
  Serial.print(" master=");
  Serial.print((int)f.master);
  Serial.print(" slave=");
  Serial.print((int)f.slave);
  Serial.print(" cmd=");
  Serial.print(f.cmd);
  Serial.print(" params=");
  Serial.println(f.params);
}

void Rs485Dispatcher::serialEventState(const char* tag) {
  if (!safeBool(_cfg.debug, false)) return;
  if (!_board || !_safety || !_homing || !_motion) return;

  Serial.print("[STATE] ");
  Serial.print(tag);

  Serial.print(" | fault=");
  Serial.print(_safety->isFault() ? 1 : 0);
  Serial.print(" err=");
  Serial.print((int)_safety->getErrorCode());

  Serial.print(" | homing=");
  Serial.print(_homing->isActive() ? 1 : 0);
  Serial.print(" ref=");
  Serial.print(_homing->isReferenced() ? 1 : 0);

  Serial.print(" | posActive=");
  Serial.print(_motion->isPosActive() ? 1 : 0);

  Serial.print(" | ENDL=");
  Serial.print(_board->readEndLeft() ? 1 : 0);
  Serial.print(" ENDR=");
  Serial.print(_board->readEndRight() ? 1 : 0);

  Serial.println();
}

// ============================================================================
// Private: Homing Kick (Initialisierung)
// ============================================================================

void Rs485Dispatcher::requestHomingKick(uint32_t nowMs, const char* reason) {
  // Das ist eine "Anschub"-Logik:
  // - tries/nextMs werden gesetzt
  // - die eigentliche Retry-Logik laeuft ueber updateHomingKickRetry()

  if (!(_cfg.homingKickTries && _cfg.homingKickNextMs && _cfg.homingKickMaxTries)) return;

  if (safeBool(_cfg.debug, false)) {
    Serial.print("[HOMING] requestKick reason=");
    Serial.println(reason);
  }

  *_cfg.homingKickTries = safeU8(_cfg.homingKickMaxTries, 0);

  // Wie im alten .ino: sofort moeglich (next = now)
  *_cfg.homingKickNextMs = nowMs;
}

// ============================================================================
// Private: Parser
// ============================================================================

int32_t Rs485Dispatcher::parseDeg01FromParam(const String& p) const {
  String s = p;
  s.trim();
  s.replace(",", ".");

  float f = s.toFloat();
  int32_t d01 = (int32_t)lroundf(f * 100.0f);

  // Clamping auf aktuelle Achsgrenzen
  if (_cfg.axisMinDeg01 && d01 < *_cfg.axisMinDeg01) d01 = *_cfg.axisMinDeg01;
  if (_cfg.axisMaxDeg01 && d01 > *_cfg.axisMaxDeg01) d01 = *_cfg.axisMaxDeg01;

  return d01;
}

// Hilfsparser fuer Zahlen/Booleans aus RS485-Parametern.
// - Akzeptiert ',' oder '.' als Dezimaltrenner.
// - Bool: "1/0", "true/false", "on/off".
static float parseFloatParam(const String& p) {
  String s = p;
  s.trim();
  s.replace(",", ".");
  return s.toFloat();
}

static uint32_t parseU32Param(const String& p) {
  String s = p;
  s.trim();
  if (s.length() == 0) return 0;
  return (uint32_t)strtoul(s.c_str(), nullptr, 10);
}

static int32_t parseI32Param(const String& p) {
  String s = p;
  s.trim();
  if (s.length() == 0) return 0;
  return (int32_t)strtol(s.c_str(), nullptr, 10);
}

static uint8_t parseU8Param(const String& p) {
  uint32_t v = parseU32Param(p);
  if (v > 255) v = 255;
  return (uint8_t)v;
}

static bool parseBoolParam(const String& p) {
  String s = p;
  s.trim();
  s.toLowerCase();
  if (s == "1" || s == "true" || s == "on" || s == "yes") return true;
  if (s == "0" || s == "false" || s == "off" || s == "no") return false;
  // Fallback: alles != 0 als true
  return (s.toInt() != 0);
}

static int32_t parseDeg01NoClamp(const String& p) {
  String s = p;
  s.trim();
  s.replace(",", ".");
  float f = s.toFloat();
  return (int32_t)lroundf(f * 100.0f);
}


static String formatFloatComma(float v, uint8_t decimals) {
  char buf[32];
  // snprintf mit * ist auf ESP32 ok
  snprintf(buf, sizeof(buf), "%.*f", (int)decimals, (double)v);
  String s(buf);
  s.replace(".", ",");
  return s;
}


// ============================================================================
// RS485-Checksum-Helfer (nur fuer Debug/NAK bei BADCHK)
// ============================================================================
//
// Das Protokoll erwartet als "chk" eine Dezimalzahl mit optional 2 Nachkommastellen.
// Intern wird sie als "scaled100" (Wert * 100) behandelt.
// Die Checksumme ist kompatibel zu Rs485Proto::computeChecksumScaled100():
//   chk_scaled = (src + dst) * 100 + abs(value_scaled)
// value_scaled kommt aus dem letzten Token in params nach ':' (z.B. "X:50" -> 50).
//
// Vorteil: Wenn der Master eine falsche chk sendet, koennen wir ein NAK mit der
// erwarteten Checksumme schicken, damit das Debugging einfacher wird.

static bool parseDecimalScaled100Local(const String& sIn, int32_t& outScaled) {
  String s = sIn;
  s.trim();
  if (s.length() == 0) return false;

  // Vorzeichen
  bool neg = false;
  if (s[0] == '-') {
    neg = true;
    s = s.substring(1);
  } else if (s[0] == '+') {
    s = s.substring(1);
  }
  s.trim();
  if (s.length() == 0) return false;

  // Trenner ',' oder '.'
  int p = s.indexOf(',');
  if (p < 0) p = s.indexOf('.');

  String a = (p >= 0) ? s.substring(0, p) : s;
  String b = (p >= 0) ? s.substring(p + 1) : String("");

  a.trim();
  b.trim();

  if (a.length() == 0) a = "0";

  for (uint16_t i = 0; i < a.length(); i++) {
    char ch = a[i];
    if (!(ch >= '0' && ch <= '9')) return false;
  }
  for (uint16_t i = 0; i < b.length(); i++) {
    char ch = b[i];
    if (!(ch >= '0' && ch <= '9')) return false;
  }

  long intPart = a.toInt();

  long frac = 0;
  if (b.length() == 0) {
    frac = 0;
  } else if (b.length() == 1) {
    frac = b.toInt() * 10L;
  } else {
    // mindestens 2 Stellen: nur die ersten 2 nehmen (deterministisch, kein Runden)
    String b2 = b.substring(0, 2);
    frac = b2.toInt();
  }

  long scaled = intPart * 100L + frac;
  if (neg) scaled = -scaled;

  outScaled = (int32_t)scaled;
  return true;
}

static int32_t extractValueScaled100Local(const String& params) {
  String p = params;
  p.trim();
  if (p.length() == 0) return 0;

  // letzten Token nach ':' nehmen (damit "L:25" -> 25)
  int lastColon = p.lastIndexOf(':');
  String token = (lastColon >= 0) ? p.substring(lastColon + 1) : p;
  token.trim();

  // Wenn params eine Liste enthaelt (z.B. "1;2;3"),
  // nehmen wir fuer die Checksumme das letzte Listenelement.
  while (token.length() > 0 && token.charAt(token.length() - 1) == ';') {
    token.remove(token.length() - 1);
    token.trim();
  }
  int lastSemi = token.lastIndexOf(';');
  if (lastSemi >= 0) {
    token = token.substring(lastSemi + 1);
    token.trim();
  }

  int32_t vScaled = 0;
  if (!parseDecimalScaled100Local(token, vScaled)) return 0;

  if (vScaled < 0) vScaled = -vScaled;
  return vScaled;
}

static int32_t computeChecksumScaled100Local(uint8_t src, uint8_t dst, const String& params) {
  const int32_t v = extractValueScaled100Local(params);
  const int32_t sum = ((int32_t)src + (int32_t)dst) * 100 + v;
  return sum;
}

static String formatScaled100Local(int32_t scaled) {
  // Ausgabe: ohne Nachkommastellen wenn .00, sonst mit 2 Nachkommastellen (Komma)
  if (scaled < 0) scaled = -scaled;
  int32_t ip = scaled / 100;
  int32_t fr = scaled % 100;

  if (fr == 0) {
    return String(ip);
  }

  char buf[32];
  snprintf(buf, sizeof(buf), "%ld,%02ld", (long)ip, (long)fr);
  return String(buf);
}

static bool floatChanged(float a, float b, float eps = 0.0005f) {
  return fabsf(a - b) > eps;
}


String Rs485Dispatcher::formatDeg01ToString(int32_t deg01) const {
  // Immer mit 2 Nachkommastellen, Dezimaltrenner ',' (wie im RS485-Protokoll genutzt)
  // Beispiel: 12345 -> "123,45"
  bool neg = (deg01 < 0);
  int32_t a = neg ? -deg01 : deg01;
  int32_t ip = a / 100;
  int32_t fp = a % 100;

  char buf[32];
  snprintf(buf, sizeof(buf), "%s%ld,%02ld", neg ? "-" : "", (long)ip, (long)fp);
  return String(buf);
}

// ============================================================================
// Private: Command Dispatcher
// ============================================================================


void Rs485Dispatcher::handleCommand(const Rs485Frame& f, uint32_t nowMs) {
  if (!_rs485 || !_safety || !_homing || !_motion) return;

  const uint8_t ownId = safeU8(_cfg.ownSlaveId, 0);

  // Adressierung: nur reagieren, wenn das Frame fuer uns ist.
  if (!((f.slave == ownId) || (f.slave == 255))) return;

  // Broadcast: zur Sicherheit NICHT antworten (sonst Kollision, wenn spaeter mehrere Slaves existieren)
  const bool shouldReply = (f.slave != 255);

  String cmd = f.cmd;
  cmd.trim();
  cmd.toUpperCase();

  // Deadman/Keepalive (Joerg):
  // - Es soll NUR dann als Lebenszeichen zaehlen, wenn ein Frame an UNSERE
  //   eigene ID adressiert ist.
  // - Broadcast (255) soll den Deadman NICHT "am Leben halten", weil sonst
  //   auch fremde/unsaubere Bus-Telegramme oder allgemeine Broadcasts eine
  //   Bewegung unbegrenzt lauffaehig halten koennten.
  if (f.slave == ownId) {
    _lastGetPosCmdMs = nowMs;
  }


  // Preferences: nur schreiben, wenn ein SET wirklich etwas aendert.
  Preferences* prefs = _cfg.prefs;

  // ------------------------------------------------------------------------
  // Kleine Helper fuer SET/GET-Parameter
  // ------------------------------------------------------------------------
  auto clampPercent = [](float v) -> float {
    if (v < 0.0f) return 0.0f;
    if (v > 100.0f) return 100.0f;
    return v;
  };

  auto persistPutU8 = [&](const char* key, uint8_t* var, uint8_t nv) {
    if (!var) return;
    if (*var == nv) return;                 // keine Aenderung -> kein Flash-Write
    *var = nv;
    if (prefs) prefs->putUChar(key, nv);
  };

  auto persistPutI32 = [&](const char* key, int32_t* var, int32_t nv) {
    if (!var) return;
    if (*var == nv) return;
    *var = nv;
    if (prefs) prefs->putInt(key, nv);
  };

  auto persistPutU32 = [&](const char* key, uint32_t* var, uint32_t nv) {
    if (!var) return;
    if (*var == nv) return;
    *var = nv;
    if (prefs) prefs->putUInt(key, nv);
  };

  auto persistPutBool = [&](const char* key, bool* var, bool nv) {
    if (!var) return;
    if (*var == nv) return;
    *var = nv;
    if (prefs) prefs->putBool(key, nv);
  };

  auto persistPutFloat = [&](const char* key, float* var, float nv) {
    if (!var) return;
    if (!floatChanged(*var, nv)) return;
    *var = nv;
    if (prefs) prefs->putFloat(key, nv);
  };

  // SafetyConfig live aktualisieren (ohne Neustart).
  auto applySafetyUpdate = [&]() {
    if (!_safety) return;
    SafetyConfig c = _safety->getConfig();

    if (_cfg.cmdTimeoutMs)   c.cmdTimeoutMs = *_cfg.cmdTimeoutMs;
    if (_cfg.isSoftWarnMv)   c.isSoftWarnMv = *_cfg.isSoftWarnMv;
    if (_cfg.isHardStopMv)   c.isHardStopMv = *_cfg.isHardStopMv;

    // Details Stromueberwachung
    if (_cfg.isGraceMs)      c.isGraceMs = *_cfg.isGraceMs;
    if (_cfg.isHardHoldMs)   c.isHardHoldMs = *_cfg.isHardHoldMs;
    if (_cfg.isFilterLen) {
      uint8_t fl = *_cfg.isFilterLen;
      if (fl < 1) fl = 1;
      if (fl > 32) fl = 32;
      c.isFilterLen = fl;
    }

    // Stall-Erkennung
    if (_cfg.stallMonitorEnabled) c.stallMonitorEnabled = *_cfg.stallMonitorEnabled;
    if (_cfg.minStallPwm)         c.stallArmDutyAbs = *_cfg.minStallPwm;
    if (_cfg.stallTimeoutMs) c.stallTimeoutMs = *_cfg.stallTimeoutMs;
    if (_cfg.stallMinCounts)      c.stallMinCounts = *_cfg.stallMinCounts;

    _safety->updateConfig(c);
  };

  // HomingConfig live aktualisieren (ohne Neustart).
  auto applyHomingUpdate = [&]() {
    if (!_homing) return;

    // WICHTIG:
    // Wir wollen bei partiellen RS485-Updates nicht alle anderen Homing-Parameter
    // (z.B. Rampenlaengen in Counts) auf Defaults zuruecksetzen.
    HomingConfig hc = _homing->getConfig();

    hc.fastPwmPercent = (_cfg.homeFastPwmPercent) ? *_cfg.homeFastPwmPercent : hc.fastPwmPercent;

    // SEEK-MIN (konstante PWM nach der Anfahr-Rampe)
    hc.seekMinPwmPercent = (_cfg.homeSeekMinPwmPercent) ? *_cfg.homeSeekMinPwmPercent : hc.seekMinPwmPercent;

    // Approach (langsam) soll identisch zu g_minPwm bleiben.
    // Backoff bleibt separat konfigurierbar.
    if (_cfg.minPwm) {
      hc.approachPwmPercent = *_cfg.minPwm;
    }
    const float back = (_cfg.homeBackoff) ? *_cfg.homeBackoff : hc.backoffPwmPercent;
    hc.backoffPwmPercent = back;

    hc.returnToZero     = (_cfg.homeReturnToZero) ? *_cfg.homeReturnToZero : hc.returnToZero;
    hc.segmentTimeoutMs = (_cfg.homeTimeoutMs) ? *_cfg.homeTimeoutMs : hc.segmentTimeoutMs;

    _homing->updateConfig(hc);
  };

  // Neustart anfordern (der eigentliche ESP.restart() passiert in loop()).
  // Wird genutzt, wenn Parameter geaendert werden, die nur beim setup() sauber
  // uebernommen werden koennen (z.B. EncoderType / ExpectedCounts).
  auto requestRestart = [&](uint32_t delayMs) {
    if (_cfg.restartRequested) *_cfg.restartRequested = true;
    if (_cfg.restartAtMs) *_cfg.restartAtMs = nowMs + delayMs;
  };

  // ------------------------------------------------------------------------
  // TEST
  // ------------------------------------------------------------------------
  if (cmd == "TEST") {
    if (shouldReply) sendAck(f.master, "TEST", "1");
    return;
  }

  // ------------------------------------------------------------------------
  // RESET (Werksreset / NVS loeschen)
  // - Loescht ALLE Keys im Preferences-Namespace "rotor" (Konfiguration + Baseline/Statistik)
  // - Danach IMMER Reboot
  //
  // Nutzung (Master -> Slave):
  //   RESET:1
  // (params wird fuer die Funktion ignoriert, dient nur der Checksumme)
  // ------------------------------------------------------------------------
  if (cmd == "RESET") {
    Preferences* prefs = _cfg.prefs;
    if (!prefs) {
      if (shouldReply) sendNak(f.master, "RESET", "NOPREFS");
      return;
    }

    // Alles loeschen (inkl. Kalibrier-Baseline und Live-Statistik)
    prefs->clear();

    // ACK vor dem Neustart senden (damit Master die Aktion bestaetigt bekommt)
    if (shouldReply) sendAck(f.master, "RESET", "1");

    // Neustart leicht verzoegert, damit RS485 ACK sicher rausgeht
    requestRestart(300);
    serialEventState("RESET");
    return;
  }

  // ------------------------------------------------------------------------
  // STOP
  // - Soll sauber auslaufen: Ziel wird auf Stop-Punkt (aktuelle Position) gesetzt.
  // - Der Regler bremst damit mit der aktuellen Rampe ab, rollt ggf. aus und
  //   faehrt dann wieder zurueck (Grobprofil + Feinphase).
  // ------------------------------------------------------------------------
  if (cmd == "STOP") {
    // Hinweis: Fehler (ERR) werden nur bei SETREF quittiert.
    // Warnungen werden nur ueber DELWARN geloescht.

    // Waehrend Homing: Stop-Punkt macht keinen Sinn -> Homing abbrechen + weich stoppen
    if (_homing->isActive()) {
      _homing->abort();
      _motion->commandStopSoft();
    } else {
      _motion->commandStopToCurrentPosition(nowMs);
    }

    // Laufende Live-Statistik / Kalibrierung abbrechen
    if (_loadMon) {
      _loadMon->abortLiveMoveTracking();
      if (_loadMon->isCalibrationRunning()) {
        _loadMon->abortCalibration(nowMs);
      }
    }

    if (shouldReply) sendAck(f.master, "STOP", "1");
    serialEventState("STOP");
    return;
  }

  // ------------------------------------------------------------------------
  // NSTOP (Not-Aus)
  // - Sofort alles auf 0, keine Rampe.
  // - Latcht einen Safety-Fault, damit der Motor garantiert aus bleibt.
  // ------------------------------------------------------------------------
  if (cmd == "NSTOP") {
    _homing->abort();
    _motion->commandStopSoft();

    _safety->triggerEmergencyStop(SE_NSTOP_CMD);

    // Laufende Live-Statistik / Kalibrierung abbrechen
    if (_loadMon) {
      _loadMon->abortLiveMoveTracking();
      if (_loadMon->isCalibrationRunning()) {
        _loadMon->abortCalibration(nowMs);
      }
    }

    if (shouldReply) sendAck(f.master, "NSTOP", "1");
    serialEventState("NSTOP");
    return;
  }

  // ------------------------------------------------------------------------
  // GETREF
  // ------------------------------------------------------------------------
  if (cmd == "GETREF") {
    if (shouldReply) sendAck(f.master, "GETREF", _homing->isReferenced() ? "1" : "0");
    return;
  }

  // ------------------------------------------------------------------------
  // GETPOSDG (Keepalive + Positionsrueckmeldung)
  // ------------------------------------------------------------------------
  if (cmd == "GETPOSDG") {

    int32_t curDeg01 = 0;
    if (!_motion->getCurrentPositionDeg01(curDeg01)) {
      if (shouldReply) sendNak(f.master, "GETPOSDG", "NOPOS");
      return;
    }

    if (shouldReply) sendAck(f.master, "GETPOSDG", formatDeg01ToString(curDeg01));
    return;
  }

  // ------------------------------------------------------------------------
  // GETIS (Strommesswert, automatisch Kanal waehlen)
  // ------------------------------------------------------------------------
  // Liefert genau EINEN Wert (mV) NACH Offset-Abzug.
  // Kanalwahl:
  //   - positive Bewegung (PWM > 0)  -> IS1
  //   - negative Bewegung (PWM < 0)  -> IS2
  //   - Stillstand (PWM == 0)        -> letzter Nicht-Null-Richtung (Fallback), sonst IS1
  //
  // Hintergrund:
  // - Beim BTN8982 ist der Strommess-Ausgang je nach Stromflussrichtung auf IS1 oder IS2 aktiv.
  // - Der Master soll IMMER nur den aktuell relevanten Wert bekommen.
  // - Wir ziehen hier bewusst den konfigurierten Offset ab (AutoOffset + Trim), damit
  //   der Master keine Rohwerte interpretieren muss.
  // ------------------------------------------------------------------------
  if (cmd == "GETIS") {
    if (!_board || !_safety) {
      if (shouldReply) sendNak(f.master, "GETIS", "NOHW");
      return;
    }

    // Richtung aus der zuletzt von Safety freigegebenen PWM ableiten
    int8_t dir = _safety->getLastOutputDutySign();
    if (dir == 0) dir = _safety->getLastOutputDutySignNonZero();

    // ADC lesen (mV)
    const uint32_t adc1 = _board->readIs1mV();
    const uint32_t adc2 = _board->readIs2mV();

    // Offset (mV) aus SafetyConfig holen und abziehen
    const SafetyConfig sc = _safety->getConfig();
    const uint32_t off1 = sc.isOffset1Mv;
    const uint32_t off2 = sc.isOffset2Mv;

    uint32_t is1 = (adc1 > off1) ? (adc1 - off1) : 0;
    uint32_t is2 = (adc2 > off2) ? (adc2 - off2) : 0;

    // Kanal waehlen
    const uint32_t val = (dir < 0) ? is2 : is1;

    // Antwort: reiner Zahlenwert in mV (Offset bereits abgezogen)
    if (shouldReply) sendAck(f.master, "GETIS", String(val));
    return;
  }

  // ------------------------------------------------------------------------

  // ------------------------------------------------------------------------
  // GETTEMPA / GETTEMPM (Temperaturen in GradC)
  // ------------------------------------------------------------------------
  // Liefert die zuletzt gemessene Temperatur (DS18B20) in GradC.
  // - GETTEMPA: Umgebung
  // - GETTEMPM: Motor
  //
  // Format:
  //   #<DEV>:<MASTER>:ACK_GETTEMPA:<TEMP_C>:<CS>$
  //   #<DEV>:<MASTER>:ACK_GETTEMPM:<TEMP_C>:<CS>$
  //
  // Hinweis:
  // - Wenn der Motor-Sensor deaktiviert ist, liefert GETTEMPM immer 0.
  // ------------------------------------------------------------------------
  if (cmd == "GETTEMPA") {
    if (!_temps || !_rs485) {
      if (shouldReply) sendNak(f.master, "GETTEMPA", "NOTEMP");
      return;
    }

    const int32_t tScaled = _temps->getAmbientScaled100();
    if (shouldReply) sendAck(f.master, "GETTEMPA", _rs485->formatScaled100(tScaled));
    return;
  }

  if (cmd == "GETTEMPM") {
    if (!_temps || !_rs485) {
      if (shouldReply) sendNak(f.master, "GETTEMPM", "NOTEMP");
      return;
    }

    const int32_t tScaled = _temps->getMotorScaled100();
    if (shouldReply) sendAck(f.master, "GETTEMPM", _rs485->formatScaled100(tScaled));
    return;
  }


  // ------------------------------------------------------------------------
  // GETSWAPTMP / SETSWAPTEMP (Temperatur-Sensoren tauschen)
  // ------------------------------------------------------------------------
  // Motivation:
  // - DS18B20 werden am OneWire-Bus nicht immer in gleicher Reihenfolge gefunden.
  // - Je nach verbauten Sensoren kann "Device 0" mal der Motor- und mal der
  //   Umgebungs-Sensor sein.
  //
  // Loesung:
  // - Ein persistenter Swap-Schalter tauscht die logische Zuordnung:
  //   * Swap=0: Umgebung=Device0, Motor=Device1
  //   * Swap=1: Umgebung=Device1, Motor=Device0
  //
  // Wichtig:
  // - Die Limits/Warnungen (twa/twm) bleiben *logisch* gleich.
  //   Durch Swap landen sie am richtigen physikalischen Sensor.
  //
  // Format:
  //   #<DEV>:<MASTER>:ACK_GETSWAPTMP:<0|1>:<CS>$
  //   #<DEV>:<MASTER>:ACK_SETSWAPTEMP:1:<CS>$
  //
  // Speicherung:
  //   Preferences Key: "tsw" (bool)
  // ------------------------------------------------------------------------
  if (cmd == "GETSWAPTMP") {
    const bool v = (_cfg.tempSwapSensors) ? *_cfg.tempSwapSensors : false;
    if (shouldReply) sendAck(f.master, "GETSWAPTMP", v ? "1" : "0");
    return;
  }

  if (cmd == "SETSWAPTEMP") {
    const bool v = parseBoolParam(f.params);

    // Persistenter Wert aktualisieren
    persistPutBool("tsw", _cfg.tempSwapSensors, v);

    // Sofort anwenden (damit GETTEMPA/GETTEMPM und Warnungen direkt passen)
    if (_temps) {
      _temps->setSwapTemp(v);
    }

    if (shouldReply) sendAck(f.master, "SETSWAPTEMP", "1");
    serialEventState("SETSWAPTEMP");
    return;
  }

  // ------------------------------------------------------------------------
  // GETANEMO (Windgeschwindigkeit, km/h)
  // ------------------------------------------------------------------------
  // ALT: analog 0..2V an IO9.
  // NEU: RS485 Wind-/Richtungssensor (Modbus) via HalBoard Cache.
  // Rueckgabe bleibt kompatibel: km/h mit 1 Nachkommastelle (Komma).
  // Offset (km/h) wird wie frueher in Preferences (Key: "ano") gespeichert.
  // ------------------------------------------------------------------------
  if (cmd == "GETANEMO") {
    if (!_board) {
      if (shouldReply) sendNak(f.master, "GETANEMO", "NOHW");
      return;
    }

    // Wenn der Wind-/Richtungssensor deaktiviert ist, liefern wir immer 0
    // (und vermeiden jede Abfrage ueber RS485/Modbus).
    if (_board && !_board->getWindEnable()) {
      if (shouldReply) sendAck(f.master, "GETANEMO", formatFloatComma(0.0f, 1));
      return;
    }

    float kmh = 0.0f;
    if (!_board->getWindSpeedKmh(kmh)) {
      if (shouldReply) sendNak(f.master, "GETANEMO", "NOWIND");
      return;
    }

    // Safety-Clamp (nur um offensichtliche Ausreisser zu begrenzen)
    if (!isfinite(kmh)) kmh = 0.0f;
    if (kmh < 0.0f) kmh = 0.0f;
    if (kmh > 250.0f) kmh = 250.0f;

    if (shouldReply) sendAck(f.master, "GETANEMO", formatFloatComma(kmh, 1));
    return;
  }


  // ------------------------------------------------------------------------
  // GETBEAUFORT (Windstaerke in Beaufort 0..12)
  // ------------------------------------------------------------------------
  if (cmd == "GETBEAUFORT") {
    if (!_board) {
      if (shouldReply) sendNak(f.master, "GETBEAUFORT", "NOHW");
      return;
    }

    if (_board && !_board->getWindEnable()) {
      if (shouldReply) sendAck(f.master, "GETBEAUFORT", "0");
      return;
    }

    uint8_t bft = 0;
    if (!_board->getWindBeaufort(bft)) {
      if (shouldReply) sendNak(f.master, "GETBEAUFORT", "NOWIND");
      return;
    }

    char tmp[8];
    snprintf(tmp, sizeof(tmp), "%u", (unsigned)bft);
    if (shouldReply) sendAck(f.master, "GETBEAUFORT", tmp);
    return;
  }
  // ------------------------------------------------------------------------
  // SETANEMO (Offset in km/h setzen)
  // ------------------------------------------------------------------------
  // Parameter: +/- Offset in km/h, mit 1 Nachkommastelle (z.B. -1,5).
  // Wird in Preferences gespeichert (Key: "ano").
  // ------------------------------------------------------------------------
  if (cmd == "SETANEMOOF") {
    float v = parseFloatParam(f.params);

    // Auf 0.1 km/h quantisieren (damit Speicherung stabil bleibt)
    v = roundf(v * 10.0f) / 10.0f;

    // Sinnvolle Begrenzung, damit man keinen Unsinn speichert
    if (v < -200.0f) v = -200.0f;
    if (v > 200.0f)  v = 200.0f;

    persistPutFloat("ano", _cfg.anemoOffsetKmh, v);

    // Sofort in den Wind-Sensor uebernehmen
    if (_board) {
      _board->setWindSpeedOffsetKmh(v);
    }

    if (shouldReply) sendAck(f.master, "SETANEMOOF", "1");
    serialEventState("SETANEMOOF");
    return;
  }

  // ------------------------------------------------------------------------
  // GETANEMOOF (Offset der Windgeschwindigkeit in km/h lesen)
  // ------------------------------------------------------------------------
  // Rueckgabe: Offset in km/h mit 1 Nachkommastelle (Komma), z.B. -1,5
  // Der Wert wurde via SETANEMO im NVS (Key: "ano") gespeichert
  // und liegt zur Laufzeit in _cfg.anemoOffsetKmh.
  // ------------------------------------------------------------------------
  if (cmd == "GETANEMOOF") {
    const float v = (_cfg.anemoOffsetKmh) ? *_cfg.anemoOffsetKmh : 0.0f;
    if (shouldReply) sendAck(f.master, "GETANEMOOF", formatFloatComma(v, 1));
    return;
  }

  // ------------------------------------------------------------------------
  // WINDDIR (Windrichtung in Grad)
  // ------------------------------------------------------------------------
  // Rueckgabe: Grad mit 1 Nachkommastelle (Komma), Bereich 0..359,9
  // ------------------------------------------------------------------------
  if (cmd == "GETWINDDIR") {
    if (!_board) {
      if (shouldReply) sendNak(f.master, "WINDDIR", "NOHW");
      return;
    }

    // Wenn deaktiviert: immer 0 liefern.
    if (_board && !_board->getWindEnable()) {
      if (shouldReply) sendAck(f.master, "WINDDIR", formatFloatComma(0.0f, 1));
      return;
    }

    float deg = 0.0f;
    if (!_board->getWindDirDeg(deg)) {
      if (shouldReply) sendNak(f.master, "WINDDIR", "NOWIND");
      return;
    }

    // Normalisieren (nur zur Sicherheit)
    while (deg < 0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;

    if (shouldReply) sendAck(f.master, "WINDDIR", formatFloatComma(deg, 1));
    return;
  }

  // ------------------------------------------------------------------------
  // SETWINDDIROF (Richtungs-Offset in Grad setzen)
  // ------------------------------------------------------------------------
  // Parameter: +/- Offset in Grad (z.B. -12,3).
  // Wird in Preferences gespeichert (Key: "wdo") und sofort angewendet.
  // ------------------------------------------------------------------------
  if (cmd == "SETWINDDIROF") {
    float v = parseFloatParam(f.params);

    // Auf 0.1 Grad quantisieren
    v = roundf(v * 10.0f) / 10.0f;

    // Sinnvolle Begrenzung
    if (v < -360.0f) v = -360.0f;
    if (v >  360.0f) v =  360.0f;

    persistPutFloat("wdo", _cfg.windDirOffsetDeg, v);

    // Sofort anwenden
    if (_board) {
      _board->setWindDirOffsetDeg(v);
    }

    if (shouldReply) sendAck(f.master, "SETWINDDIROF", "1");
    serialEventState("SETWINDDIROF");
    return;
  }

  // ------------------------------------------------------------------------
  // GETWINDDIROF (Richtungs-Offset in Grad lesen)
  // ------------------------------------------------------------------------
  // Rueckgabe: Offset in Grad mit 1 Nachkommastelle (Komma), z.B. -12,3
  // Der Wert wurde via SETWINDDIROF im NVS (Key: "wdo") gespeichert
  // und liegt zur Laufzeit in _cfg.windDirOffsetDeg.
  // ------------------------------------------------------------------------
  if (cmd == "GETWINDDIROF") {
    const float v = (_cfg.windDirOffsetDeg) ? *_cfg.windDirOffsetDeg : 0.0f;
    if (shouldReply) sendAck(f.master, "GETWINDDIROF", formatFloatComma(v, 1));
    return;
  }

  // ------------------------------------------------------------------------
  // Antennen-Versatz 1..3 (nur persistent speichern / bereitstellen)
  // ------------------------------------------------------------------------
  if (cmd == "GETANTOFF1") {
    const float v = (_cfg.antOffset1Deg) ? *_cfg.antOffset1Deg : 0.0f;
    if (shouldReply) sendAck(f.master, "GETANTOFF1", formatFloatComma(v, 1));
    return;
  }
  if (cmd == "SETANTOFF1") {
    float v = parseFloatParam(f.params);
    if (!isfinite(v)) v = 0.0f;
    if (v < 0.0f) v = 0.0f;
    if (v > 360.0f) v = 360.0f;
    v = roundf(v * 10.0f) / 10.0f;
    persistPutFloat("ao1", _cfg.antOffset1Deg, v);
    if (shouldReply) sendAck(f.master, "SETANTOFF1", "1");
    serialEventState("SETANTOFF1");
    return;
  }

  if (cmd == "GETANTOFF2") {
    const float v = (_cfg.antOffset2Deg) ? *_cfg.antOffset2Deg : 0.0f;
    if (shouldReply) sendAck(f.master, "GETANTOFF2", formatFloatComma(v, 1));
    return;
  }
  if (cmd == "SETANTOFF2") {
    float v = parseFloatParam(f.params);
    if (!isfinite(v)) v = 0.0f;
    if (v < 0.0f) v = 0.0f;
    if (v > 360.0f) v = 360.0f;
    v = roundf(v * 10.0f) / 10.0f;
    persistPutFloat("ao2", _cfg.antOffset2Deg, v);
    if (shouldReply) sendAck(f.master, "SETANTOFF2", "1");
    serialEventState("SETANTOFF2");
    return;
  }

  if (cmd == "GETANTOFF3") {
    const float v = (_cfg.antOffset3Deg) ? *_cfg.antOffset3Deg : 0.0f;
    if (shouldReply) sendAck(f.master, "GETANTOFF3", formatFloatComma(v, 1));
    return;
  }
  if (cmd == "SETANTOFF3") {
    float v = parseFloatParam(f.params);
    if (!isfinite(v)) v = 0.0f;
    if (v < 0.0f) v = 0.0f;
    if (v > 360.0f) v = 360.0f;
    v = roundf(v * 10.0f) / 10.0f;
    persistPutFloat("ao3", _cfg.antOffset3Deg, v);
    if (shouldReply) sendAck(f.master, "SETANTOFF3", "1");
    serialEventState("SETANTOFF3");
    return;
  }

  // ------------------------------------------------------------------------
  // Frei gespeicherte Winkel 1..3 (nur persistent speichern / bereitstellen)
  // ------------------------------------------------------------------------
  if (cmd == "GETANGLE1") {
    const float v = (_cfg.angle1Deg) ? *_cfg.angle1Deg : 0.0f;
    if (shouldReply) sendAck(f.master, "GETANGLE1", formatFloatComma(v, 1));
    return;
  }
  if (cmd == "SETANGLE1") {
    float v = parseFloatParam(f.params);
    if (!isfinite(v)) v = 0.0f;
    if (v < 0.0f) v = 0.0f;
    if (v > 360.0f) v = 360.0f;
    v = roundf(v * 10.0f) / 10.0f;
    persistPutFloat("ag1", _cfg.angle1Deg, v);
    if (shouldReply) sendAck(f.master, "SETANGLE1", "1");
    serialEventState("SETANGLE1");
    return;
  }

  if (cmd == "GETANGLE2") {
    const float v = (_cfg.angle2Deg) ? *_cfg.angle2Deg : 0.0f;
    if (shouldReply) sendAck(f.master, "GETANGLE2", formatFloatComma(v, 1));
    return;
  }
  if (cmd == "SETANGLE2") {
    float v = parseFloatParam(f.params);
    if (!isfinite(v)) v = 0.0f;
    if (v < 0.0f) v = 0.0f;
    if (v > 360.0f) v = 360.0f;
    v = roundf(v * 10.0f) / 10.0f;
    persistPutFloat("ag2", _cfg.angle2Deg, v);
    if (shouldReply) sendAck(f.master, "SETANGLE2", "1");
    serialEventState("SETANGLE2");
    return;
  }

  if (cmd == "GETANGLE3") {
    const float v = (_cfg.angle3Deg) ? *_cfg.angle3Deg : 0.0f;
    if (shouldReply) sendAck(f.master, "GETANGLE3", formatFloatComma(v, 1));
    return;
  }
  if (cmd == "SETANGLE3") {
    float v = parseFloatParam(f.params);
    if (!isfinite(v)) v = 0.0f;
    if (v < 0.0f) v = 0.0f;
    if (v > 360.0f) v = 360.0f;
    v = roundf(v * 10.0f) / 10.0f;
    persistPutFloat("ag3", _cfg.angle3Deg, v);
    if (shouldReply) sendAck(f.master, "SETANGLE3", "1");
    serialEventState("SETANGLE3");
    return;
  }

  // ------------------------------------------------------------------------
  // GETWINDENABLE / SETWINDENABLE (Wind-/Richtungssensor aktivieren)
  // ------------------------------------------------------------------------
  // 1 = Sensor wird gepollt, GETANEMO/GETWINDDIR liefern echte Werte
  // 0 = keine Abfragen, GETANEMO/GETWINDDIR liefern immer 0
  //
  // Speicherung:
  //   Preferences Key: "wen" (bool)
  // ------------------------------------------------------------------------
  if (cmd == "GETWINDENABLE") {
    const bool v = (_board) ? _board->getWindEnable() : ((_cfg.windEnable) ? *_cfg.windEnable : true);
    if (shouldReply) sendAck(f.master, "GETWINDENABLE", v ? "1" : "0");
    return;
  }

  if (cmd == "SETWINDENABLE") {
    const bool v = parseBoolParam(f.params);

    // Persistenter Wert aktualisieren
    persistPutBool("wen", _cfg.windEnable, v);

    // Sofort anwenden
    if (_board) {
      _board->setWindEnable(v);
    }

    if (shouldReply) sendAck(f.master, "SETWINDENABLE", "1");
    serialEventState("SETWINDENABLE");
    return;
  }

  // ------------------------------------------------------------------------
  // GETTEMPAW / GETTEMPMW (Temperatur-Warnschwellen)
  // ------------------------------------------------------------------------
  // Liefert die aktuell eingestellte Warnschwelle in GradC.
  // <= 0 deaktiviert die jeweilige Warnung.
  // ------------------------------------------------------------------------
  if (cmd == "GETTEMPAW") {
    const float thr = (_cfg.tempWarnAmbientC) ? *_cfg.tempWarnAmbientC : 0.0f;
    if (shouldReply) sendAck(f.master, "GETTEMPAW", formatFloatComma(thr, 2));
    return;
  }

  if (cmd == "GETTEMPMW") {
    const float thr = (_cfg.tempWarnMotorC) ? *_cfg.tempWarnMotorC : 0.0f;
    if (shouldReply) sendAck(f.master, "GETTEMPMW", formatFloatComma(thr, 2));
    return;
  }

  // ------------------------------------------------------------------------
  // SETTEMPA / SETTEMPM (Temperatur-Warnschwellen setzen)
  // ------------------------------------------------------------------------
  // Speichert die Warnschwelle in Preferences.
  // ------------------------------------------------------------------------
  if (cmd == "SETTEMPA") {
    const float v = parseFloatParam(f.params);
    persistPutFloat("twa", _cfg.tempWarnAmbientC, v);
    if (shouldReply) sendAck(f.master, "SETTEMPA", "1");
    serialEventState("SETTEMPA");
    return;
  }

  if (cmd == "SETTEMPM") {
    const float v = parseFloatParam(f.params);
    persistPutFloat("twm", _cfg.tempWarnMotorC, v);
    if (shouldReply) sendAck(f.master, "SETTEMPM", "1");
    serialEventState("SETTEMPM");
    return;
  }

  // ------------------------------------------------------------------------
  // Kaelte-Kompensation (Reibung darf bei niedriger Temperatur hoeher sein)
  // ------------------------------------------------------------------------
  // SETCOLDT: Temperaturgrenze (GradC)
  // SETCOLDP: zusaetzlich erlaubte Reibung in Prozent
  // ------------------------------------------------------------------------
  if (cmd == "GETCOLDT") {
    const float v = (_cfg.coldTempDegC) ? *_cfg.coldTempDegC : 5.0f;
    if (shouldReply) sendAck(f.master, "GETCOLDT", formatFloatComma(v, 2));
    return;
  }

  if (cmd == "SETCOLDT") {
    const float v = parseFloatParam(f.params);
    persistPutFloat("cth", _cfg.coldTempDegC, v);
    if (shouldReply) sendAck(f.master, "SETCOLDT", "1");
    serialEventState("SETCOLDT");
    return;
  }

  if (cmd == "GETCOLDP") {
    const float v = (_cfg.coldExtraDragPct) ? *_cfg.coldExtraDragPct : 10.0f;
    if (shouldReply) sendAck(f.master, "GETCOLDP", formatFloatComma(v, 1));
    return;
  }

  if (cmd == "SETCOLDP") {
    const float v = parseFloatParam(f.params);
    persistPutFloat("cpx", _cfg.coldExtraDragPct, v);
    if (shouldReply) sendAck(f.master, "SETCOLDP", "1");
    serialEventState("SETCOLDP");
    return;
  }

  // ------------------------------------------------------------------------
  // Statistiken/Schwellwerte fuer LoadMonitor (wie Etappe2: ramp/cal/acc/smm)
  // ------------------------------------------------------------------------
  if (cmd == "GETCALIGNDG") {
    const float v = (_cfg.calIgnoreRampDeg) ? *_cfg.calIgnoreRampDeg : 10.0f;
    if (shouldReply) sendAck(f.master, "GETCALIGNDG", formatFloatComma(v, 2));
    return;
  }

  if (cmd == "SETCALIGNDG") {
    const float v = parseFloatParam(f.params);
    persistPutFloat("cig", _cfg.calIgnoreRampDeg, v);
    if (shouldReply) sendAck(f.master, "SETCALIGNDG", "1");
    serialEventState("SETCALIGNDG");
    return;
  }

  if (cmd == "GETSTATMINDG") {
    const float v = (_cfg.statMinMoveDeg) ? *_cfg.statMinMoveDeg : 5.0f;
    if (shouldReply) sendAck(f.master, "GETSTATMINDG", formatFloatComma(v, 2));
    return;
  }

  if (cmd == "SETSTATMINDG") {
    const float v = parseFloatParam(f.params);
    persistPutFloat("smm", _cfg.statMinMoveDeg, v);
    if (shouldReply) sendAck(f.master, "SETSTATMINDG", "1");
    serialEventState("SETSTATMINDG");
    return;
  }

  if (cmd == "GETRAPDG") {
    const float v = (_cfg.accIgnoreRampDeg) ? *_cfg.accIgnoreRampDeg : 5.0f;
    if (shouldReply) sendAck(f.master, "GETRAPDG", formatFloatComma(v, 2));
    return;
  }

  if (cmd == "SETRAPDG") {
    float v = parseFloatParam(f.params);
    if (v < 0.0f) v = 0.0f;
    persistPutFloat("rap", _cfg.accIgnoreRampDeg, v);
    if (shouldReply) sendAck(f.master, "SETRAPDG", "1");
    serialEventState("SETRAPDG");
    return;
  }

  if (cmd == "GETDRAG") {
    const float v = (_cfg.dragWarnPct) ? *_cfg.dragWarnPct : 25.0f;
    if (shouldReply) sendAck(f.master, "GETDRAG", formatFloatComma(v, 1));
    return;
  }

  if (cmd == "SETDRAG") {
    const float v = parseFloatParam(f.params);
    persistPutFloat("drw", _cfg.dragWarnPct, v);
    if (shouldReply) sendAck(f.master, "SETDRAG", "1");
    serialEventState("SETDRAG");
    return;
  }

  if (cmd == "GETDRAGBINS") {
    const float v = (_cfg.dragWarnBinsPct) ? *_cfg.dragWarnBinsPct : 30.0f;
    if (shouldReply) sendAck(f.master, "GETDRAGBINS", formatFloatComma(v, 1));
    return;
  }

  if (cmd == "SETDRAGBINS") {
    const float v = parseFloatParam(f.params);
    persistPutFloat("drb", _cfg.dragWarnBinsPct, v);
    if (shouldReply) sendAck(f.master, "SETDRAGBINS", "1");
    serialEventState("SETDRAGBINS");
    return;
  }

  if (cmd == "GETDRAGPERSIST") {
    const uint8_t v = (_cfg.dragPersistMoves) ? *_cfg.dragPersistMoves : 3;
    if (shouldReply) sendAck(f.master, "GETDRAGPERSIST", String(v));
    return;
  }

  if (cmd == "SETDRAGPERSIST") {
    const uint8_t v = parseU8Param(f.params);
    persistPutU8("drn", _cfg.dragPersistMoves, v);
    if (shouldReply) sendAck(f.master, "SETDRAGPERSIST", "1");
    serialEventState("SETDRAGPERSIST");
    return;
  }

  if (cmd == "GETWINDPEAK") {
    const float v = (_cfg.windPeakPct) ? *_cfg.windPeakPct : 60.0f;
    if (shouldReply) sendAck(f.master, "GETWINDPEAK", formatFloatComma(v, 1));
    return;
  }

  if (cmd == "SETWINDPEAK") {
    const float v = parseFloatParam(f.params);
    persistPutFloat("wpk", _cfg.windPeakPct, v);
    if (shouldReply) sendAck(f.master, "SETWINDPEAK", "1");
    serialEventState("SETWINDPEAK");
    return;
  }

  if (cmd == "GETWINDCOH") {
    const float v = (_cfg.windCoherenceMin) ? *_cfg.windCoherenceMin : 55.0f;
    if (shouldReply) sendAck(f.master, "GETWINDCOH", formatFloatComma(v, 1));
    return;
  }

  if (cmd == "SETWINDCOH") {
    const float v = parseFloatParam(f.params);
    persistPutFloat("wco", _cfg.windCoherenceMin, v);
    if (shouldReply) sendAck(f.master, "SETWINDCOH", "1");
    serialEventState("SETWINDCOH");
    return;
  }

  // ------------------------------------------------------------------------
  // GETCALVALID (Baseline vorhanden?)
  // ------------------------------------------------------------------------
  if (cmd == "GETCALVALID") {
    uint8_t v = 0;
    if (_loadMon && _loadMon->hasCalibration()) v = 1;
    if (shouldReply) sendAck(f.master, "GETCALVALID", String(v));
    return;
  }

  // ------------------------------------------------------------------------
  // SETCAL (Kalibrierfahrt starten)
  // ------------------------------------------------------------------------
  // Fuehrt intern eine Kalibrierfahrt aus: 0 -> 360 -> 0
  // Dabei werden geglaettete Stromwerte pro Winkel-Bin (72) gespeichert.
  //
  // Voraussetzungen:
  // - REFF muss bereits gemacht sein.
  // - Keine Homing-Aktion aktiv.
  // - Keine Positionsfahrt aktiv.
  // ------------------------------------------------------------------------
  if (cmd == "SETCAL") {
    if (!_loadMon) {
      if (shouldReply) sendNak(f.master, "SETCAL", "NOLOAD");
      return;
    }

    if (!_homing || !_homing->isReferenced()) {
      if (shouldReply) sendNak(f.master, "SETCAL", "NOREF");
      return;
    }

    if (_homing && _homing->isActive()) {
      if (shouldReply) sendNak(f.master, "SETCAL", "BUSY_HOME");
      return;
    }

    if (_motion && _motion->isPosActive()) {
      if (shouldReply) sendNak(f.master, "SETCAL", "BUSY_POS");
      return;
    }

    const bool ok = _loadMon->startCalibration(nowMs);
    if (shouldReply) {
      if (ok) sendAck(f.master, "SETCAL", "1");
      else    sendNak(f.master, "SETCAL", "BUSY");
    }

    serialEventState("SETCAL");
    return;
  }

  // ------------------------------------------------------------------------
  // ABORTCAL (Kalibrierfahrt abbrechen)
  // ------------------------------------------------------------------------
  if (cmd == "ABORTCAL") {
    if (!_loadMon) {
      if (shouldReply) sendNak(f.master, "ABORTCAL", "NOLOAD");
      return;
    }

    _loadMon->abortCalibration(nowMs);
    if (shouldReply) sendAck(f.master, "ABORTCAL", "1");
    serialEventState("ABORTCAL");
    return;
  }

  // ------------------------------------------------------------------------
  // DELCAL (gespeichertes Kalibrierprofil loeschen)
  // ------------------------------------------------------------------------
  if (cmd == "DELCAL") {
    if (!_loadMon) {
      if (shouldReply) sendNak(f.master, "DELCAL", "NOLOAD");
      return;
    }

    const bool ok = _loadMon->deleteCalibration();
    if (shouldReply) sendAck(f.master, "DELCAL", ok ? "1" : "0");
    serialEventState("DELCAL");
    return;
  }

  // ------------------------------------------------------------------------
  // CLRSTAT (laufende Statistik loeschen)
  // ------------------------------------------------------------------------
  if (cmd == "CLRSTAT") {
    if (!_loadMon) {
      if (shouldReply) sendNak(f.master, "CLRSTAT", "NOLOAD");
      return;
    }

    _loadMon->clearStats();
    if (shouldReply) sendAck(f.master, "CLRSTAT", "1");
    serialEventState("CLRSTAT");
    return;
  }

  if (cmd == "SETACCBINSRST") {
    if (!_loadMon) {
      if (shouldReply) sendNak(f.master, "SETACCBINSRST", "NOLOAD");
      return;
    }

    _loadMon->clearAccStats();
    if (shouldReply) sendAck(f.master, "SETACCBINSRST", "1");
    serialEventState("SETACCBINSRST");
    return;
  }

  // ------------------------------------------------------------------------
  // GETCALSTATE (Kalibrierstatus)
  // ------------------------------------------------------------------------
  // Antwort:
  //   #<DEV>:<MASTER>:ACK_GETCALSTATE:<STATE>;<PROGRESS>:<CS>$
  //
  // STATE:
  //   0=IDLE, 1=RUNNING, 2=DONE, 3=ABORT, 4=ERROR
  // PROGRESS: 0..100
  // ------------------------------------------------------------------------
  if (cmd == "GETCALSTATE") {
    if (!_loadMon) {
      if (shouldReply) sendNak(f.master, "GETCALSTATE", "NOLOAD");
      return;
    }

    String payload;
    payload.reserve(24);
    payload += String(_loadMon->getCalState());
    payload += ";";
    payload += String(_loadMon->getCalProgress());

    if (shouldReply) sendAck(f.master, "GETCALSTATE", payload);
    return;
  }

  // ------------------------------------------------------------------------
  // GETLOADSTAT (Summenstatistik)
  // ------------------------------------------------------------------------
  // Antwort:
  //   #<DEV>:<MASTER>:ACK_GETLOADSTAT:<MEAN_PCT>;<PEAKABS_PCT>;<COH_PCT>;<MOVES>:<CS>$
  // ------------------------------------------------------------------------
  if (cmd == "GETLOADSTAT") {
    if (!_loadMon) {
      if (shouldReply) sendNak(f.master, "GETLOADSTAT", "NOLOAD");
      return;
    }

    int16_t meanPct = 0;
    int16_t peakAbsPct = 0;
    int16_t cohPct = 0;
    uint16_t movesUsed = 0;

    _loadMon->getLoadStat(meanPct, peakAbsPct, cohPct, movesUsed);

    String payload;
    payload.reserve(48);
    payload += String(meanPct);
    payload += ";";
    payload += String(peakAbsPct);
    payload += ";";
    payload += String(cohPct);
    payload += ";";
    payload += String(movesUsed);

    if (shouldReply) sendAck(f.master, "GETLOADSTAT", payload);
    return;
  }

  // ------------------------------------------------------------------------
  // GETWIND (Wind-Schaetzung)
  // ------------------------------------------------------------------------
  // Antwort:
  //   #<DEV>:<MASTER>:ACK_GETWIND:<WINDDIR_DEG>;<PEAKDEG>;<PEAKPCT>;<COH_PCT>:<CS>$
  // ------------------------------------------------------------------------
  if (cmd == "GETWIND") {
    if (!_loadMon) {
      if (shouldReply) sendNak(f.master, "GETWIND", "NOLOAD");
      return;
    }

    uint16_t windDirDeg = 0;
    uint16_t peakDeg = 0;
    int16_t peakPct = 0;
    uint16_t cohPct = 0;

    _loadMon->getWindInfo(windDirDeg, peakDeg, peakPct, cohPct);

    String payload;
    payload.reserve(48);
    payload += String(windDirDeg);
    payload += ";";
    payload += String(peakDeg);
    payload += ";";
    payload += String(peakPct);
    payload += ";";
    payload += String(cohPct);

    if (shouldReply) sendAck(f.master, "GETWIND", payload);
    return;
  }

  // ------------------------------------------------------------------------
  // GETCALBINS / GETLIVEBINS / GETACCBINS / GETDELTABINS (Paging, 12 Werte pro Page)
  // ------------------------------------------------------------------------
  // Request Params:
  //   <DIR>;<START>;<COUNT>
  //     DIR: 1=CW(positiv/IS1)  2=CCW(negativ/IS2)
  //
  // Antwort:
  //   #<DEV>:<MASTER>:ACK_GETCALBINS:<DIR>;<START>;<COUNT>;<V0>;...;<Vn>:<CS>$
  //   #<DEV>:<MASTER>:ACK_GETLIVEBINS:<DIR>;<START>;<COUNT>;<V0>;...;<Vn>:<CS>$
  //   #<DEV>:<MASTER>:ACK_GETACCBINS:<DIR>;<START>;<COUNT>;<V0>;...;<Vn>:<CS>$
  //   #<DEV>:<MASTER>:ACK_GETDELTABINS:<DIR>;<START>;<COUNT>;<V0>;...;<Vn>:<CS>$
  //
  // Werte:
  //   - CAL/LIVE: Strom in mV
  //   - DELTA: Prozent (int), kann negativ sein
  // ------------------------------------------------------------------------
  auto parseSemi3I32 = [&](const String& in, int32_t& a, int32_t& b, int32_t& c) -> bool {
    String s = in;
    s.trim();
    if (s.length() == 0) return false;

    int p1 = s.indexOf(';');
    if (p1 < 0) return false;
    int p2 = s.indexOf(';', p1 + 1);
    if (p2 < 0) return false;

    String sa = s.substring(0, p1);
    String sb = s.substring(p1 + 1, p2);
    String sc = s.substring(p2 + 1);

    a = parseI32Param(sa);
    b = parseI32Param(sb);
    c = parseI32Param(sc);
    return true;
  };

  auto handleBins = [&](const char* cmdName, uint8_t which) {
    // which: 0=cal, 1=live, 2=acc, 3=delta
    if (!_loadMon) {
      if (shouldReply) sendNak(f.master, cmdName, "NOLOAD");
      return;
    }

    int32_t dir = 0;
    int32_t start = 0;
    int32_t count = 0;
    if (!parseSemi3I32(f.params, dir, start, count)) {
      if (shouldReply) sendNak(f.master, cmdName, "PARAM");
      return;
    }

    if (!(dir == 1 || dir == 2)) {
      if (shouldReply) sendNak(f.master, cmdName, "DIR");
      return;
    }

    if (start < 0) start = 0;
    if (start > 71) start = 71;

    if (count < 1) count = 1;
    if (count > 12) count = 12;

    // Clamping auf 72 Bins
    int32_t maxCount = 72 - start;
    if (count > maxCount) count = maxCount;

    String payload;
    payload.reserve(16 + (uint32_t)count * 7);
    payload += String(dir);
    payload += ";";
    payload += String(start);
    payload += ";";
    payload += String(count);

    for (int32_t i = 0; i < count; i++) {
      payload += ";";
      const uint8_t idx = (uint8_t)(start + i);

      if (which == 0) {
        const uint16_t v = _loadMon->getCalBin((uint8_t)dir, idx);
        payload += String(v);
      } else if (which == 1) {
        const uint16_t v = _loadMon->getLiveBin((uint8_t)dir, idx);
        payload += String(v);
      } else if (which == 2) {
        const uint16_t v = _loadMon->getAccBin((uint8_t)dir, idx);
        payload += String(v);
      } else {
        const int16_t v = _loadMon->getDeltaPct((uint8_t)dir, idx);
        payload += String(v);
      }
    }

    if (shouldReply) sendAck(f.master, cmdName, payload);
  };

  if (cmd == "GETCALBINS") {
    handleBins("GETCALBINS", 0);
    return;
  }

  if (cmd == "GETLIVEBINS") {
    handleBins("GETLIVEBINS", 1);
    return;
  }

  if (cmd == "GETACCBINS") {
    handleBins("GETACCBINS", 2);
    return;
  }

  if (cmd == "GETDELTABINS") {
    handleBins("GETDELTABINS", 3);
    return;
  }

  // GETWARN (gesammelte Warnungen)
  // ------------------------------------------------------------------------
  // Antwortformat (mit CMD, wie bei allen anderen ACKs):
  //   #<DEVICEID>:<MASTERID>:ACK_WARN:<WARNID>;<WARNID>;...:<CS>$
  //
  // Hinweise:
  // - Warnungen werden von Safety gesammelt (max 8) und NICHT automatisch geloescht.
  // - Warnungen werden NUR ueber DELWARN geloescht.
  // - Bei 0 Warnungen senden wir "0".
  // ------------------------------------------------------------------------
  if (cmd == "GETWARN") {
    String payload;
    payload.reserve(64);

    uint8_t n = 0;
    if (_safety) n = _safety->getWarnCount();

    if (n == 0) {
      payload = "0";
    } else {
      for (uint8_t i = 0; i < n; i++) {
        if (i > 0) payload += ";";
        payload += String(_safety->getWarnAt(i));
      }
    }

    if (shouldReply) {
      // Wichtig: Master sendet GETWARN, wir antworten bewusst mit ACK_WARN
      // (kurz/lesbar und konsistent zu anderen ACKs).
      const uint8_t own = safeU8(_cfg.ownSlaveId, 0);
      _rs485->sendFrame(own, f.master, "ACK_WARN", payload);
    }
    return;
  }



  // ------------------------------------------------------------------------
  // DELWARN (Warnungen loeschen)
  // ------------------------------------------------------------------------
  // Master kann Warnungen explizit quittieren.
  // Antwort:
  //   #<DEVICEID>:<MASTERID>:ACK_DELWARN:1:<CS>$
  //
  // Hinweis:
  // - Warnungen werden NICHT mehr implizit durch STOP/SETPOSDG/SETREF geloescht.
  // - Dadurch kann der Master Warnungen einsammeln und bewusst zur passenden Zeit quittieren.
  // ------------------------------------------------------------------------
  if (cmd == "DELWARN") {
    if (!_safety) {
      if (shouldReply) sendNak(f.master, "DELWARN", "NOHW");
      return;
    }

    _safety->clearWarnings();

    if (shouldReply) sendAck(f.master, "DELWARN", "1");
    serialEventState("DELWARN");
    return;
  }

  // ------------------------------------------------------------------------
  // GETERR (aktueller Fehler, falls gelatched)
  // ------------------------------------------------------------------------
  // Antwortformat (mit CMD, wie bei allen anderen ACKs):
  //   #<DEVICEID>:<MASTERID>:ACK_ERR:<ERRID>:<CS>$
  //
  // Hinweise:
  // - Ein Fehler fuehrt immer zum Stillstand -> es ist praktisch immer nur EIN Code aktiv.
  // - Wenn kein Fehler aktiv ist, senden wir "0".
  // ------------------------------------------------------------------------
  if (cmd == "GETERR") {
    uint8_t err = 0;
    if (_safety && _safety->isFault()) {
      err = _safety->getErrorCode();
    }

    if (shouldReply) {
      // Wichtig: Master sendet GETERR, wir antworten bewusst mit ACK_ERR
      // (kurz/lesbar und konsistent zu anderen ACKs).
      const uint8_t own = safeU8(_cfg.ownSlaveId, 0);
      _rs485->sendFrame(own, f.master, "ACK_ERR", String(err));
    }
    return;
  }

  // ------------------------------------------------------------------------
  // SETREF
  // ------------------------------------------------------------------------
  // Joerg: SETREF quittiert Fehler (ERR) und startet (wie frueher) das Homing.
  // Steuerung ueber Parameter:
  //   SETREF:0  -> nur ERR quittieren (kein Homing)
  //   SETREF:1  -> Homing starten (Standard/Legacy)
  // Hinweis: Der Master soll vor SETPOSDG nur dann SETREF schicken, wenn wirklich ein Fault quittiert
  // werden muss oder wenn bewusst homed werden soll.
  if (cmd == "SETREF") {
    // Fehler quittieren: laut Spezifikation werden ERR nur bei SETREF geloescht.
    // Warnungen bleiben bestehen (werden nur ueber DELWARN geloescht).
    _safety->clearFault();

    // Bei SETPOSDG werden ERR/WARN NICHT automatisch quittiert.
    // ERR nur ueber SETREF, WARN nur ueber DELWARN.
    _safety->notifyMotionEdge(nowMs);

    // Kalibrierung/Live-Tracking beenden (SETREF ist eine "harte" Zustandskante)
    if (_loadMon) {
      _loadMon->abortLiveMoveTracking();
      if (_loadMon->isCalibrationRunning()) {
        _loadMon->abortCalibration(nowMs);
      }
    }

    // Motion immer sauber stoppen/resetten
    _motion->commandClearMotionForSetRef(nowMs);

    const uint32_t mode = parseU32Param(f.params);
    const bool startHoming = (mode >= 1);

    if (startHoming) {
      // Homing starten (Legacy)
      _homing->start();

      // "Kick" state setzen, damit ggf. nochmal gestartet wird
      requestHomingKick(nowMs, "SETREF");
      serialEventState("SETREF_HOME");
    } else {
      // Nur quittieren, Referenzstatus bleibt unveraendert
      // (Wenn Homing gerade aktiv war, lassen wir es bewusst in Ruhe - dafuer gibt es STOP/NSTOP.)
      serialEventState("SETREF_ACK");
    }

    if (shouldReply) sendAck(f.master, "SETREF", "1");
    return;
  }

  // ------------------------------------------------------------------------
  // JOG ist absichtlich deaktiviert
  // ------------------------------------------------------------------------
  if (cmd == "JOG") {
    if (shouldReply) sendNak(f.master, "JOG", "DISABLED");
    return;
  }

  // ------------------------------------------------------------------------
  // SETPOSDG (Positionsfahrt in Grad)
  // ------------------------------------------------------------------------
  if (cmd == "SETPOSDG") {
    if (!_homing->isReferenced()) {
      if (shouldReply) sendNak(f.master, "SETPOSDG", "NOREF");
      serialEventState("SETPOSDG_NOREF");
      return;
    }

    // Bei SETPOSDG werden ERR/WARN NICHT automatisch quittiert.
    // ERR nur ueber SETREF, WARN nur ueber DELWARN.
    _safety->notifyMotionEdge(nowMs);

    int32_t deg01 = parseDeg01FromParam(f.params);

    int32_t startDeg01 = 0;
    if (_motion) (void)_motion->getCurrentPositionDeg01(startDeg01);

    if (!_motion->commandSetPosDeg01(deg01, nowMs)) {
      if (shouldReply) sendNak(f.master, "SETPOSDG", "NOREF");
      serialEventState("SETPOSDG_NOREF");
      return;
    }

    // Start der Bewegung fuer Live-Statistik melden (nur grosse Bewegungen werden spaeter ausgewertet)
    if (_loadMon) {
      _loadMon->notifyMoveStarted(startDeg01, deg01, nowMs);
    }

    if (shouldReply) sendAck(f.master, "SETPOSDG", "1");
    serialEventState("SETPOSDG");
    return;
  }

  // ============================================================================
  // Persistente Parameter (Preferences) + Laufzeitparameter
  // ============================================================================
  // Alle SET... Kommandos:
  // - setzen den Wert SOFORT (wirksam ohne Neustart)
  // - schreiben NUR bei Aenderung in Preferences (Flash schonen)
  // - verwenden die gleichen Keys wie loadPreferencesIntoGlobals() in der .ino
  // ============================================================================

  // ------------------------------------------------------------------------
  // GETID / SETID
  // ------------------------------------------------------------------------
  if (cmd == "GETID") {
    const uint8_t id = safeU8(_cfg.ownSlaveId, 0);
    if (shouldReply) sendAck(f.master, "GETID", String(id));
    return;
  }

  if (cmd == "SETID") {
    const uint8_t newId = parseU8Param(f.params);
    const uint8_t oldId = safeU8(_cfg.ownSlaveId, 0);

    const bool changed = (_cfg.ownSlaveId && (*_cfg.ownSlaveId != newId));
    if (changed) {
      // ACK mit alter ID senden
      if (shouldReply && _rs485) {
        _rs485->sendFrame(oldId, f.master, "ACK_SETID", "1");
      }

      // Speichern + uebernehmen
      // Persistente ID nur schreiben, wenn sie sich wirklich aendert (Flash schonen)
      persistPutU8("id", _cfg.ownSlaveId, newId);

      // Protokoll muss neue ID sofort kennen
      if (_rs485) _rs485->setOwnSlaveId(newId);

      serialEventState("SETID");
      return;
    }

    if (shouldReply) sendAck(f.master, "SETID", "1");
    serialEventState("SETID");
    return;
  }

  // ------------------------------------------------------------------------
  // Achsgrenzen (deg01)
  // ------------------------------------------------------------------------
  if (cmd == "GETBEGINDG") {
    const int32_t v = safeI32(_cfg.axisMinDeg01, 0);
    if (shouldReply) sendAck(f.master, "GETBEGINDG", formatDeg01ToString(v));
    return;
  }

  if (cmd == "SETBEGINDG") {
    int32_t newMin = parseDeg01NoClamp(f.params);

    // Konsistenz: min <= max
    int32_t curMax = safeI32(_cfg.axisMaxDeg01, newMin);
    if (newMin > curMax) curMax = newMin;

    persistPutI32("amin", _cfg.axisMinDeg01, newMin);
    persistPutI32("amax", _cfg.axisMaxDeg01, curMax);

    if (shouldReply) sendAck(f.master, "SETBEGINDG", "1");
    serialEventState("SETBEGINDG");
    return;
  }

  if (cmd == "GETMAXDG") {
    const int32_t v = safeI32(_cfg.axisMaxDeg01, 0);
    if (shouldReply) sendAck(f.master, "GETMAXDG", formatDeg01ToString(v));
    return;
  }

  if (cmd == "SETMAXDG") {
    int32_t newMax = parseDeg01NoClamp(f.params);

    // Konsistenz: min <= max
    int32_t curMin = safeI32(_cfg.axisMinDeg01, newMax);
    if (newMax < curMin) curMin = newMax;

    persistPutI32("amin", _cfg.axisMinDeg01, curMin);
    persistPutI32("amax", _cfg.axisMaxDeg01, newMax);

    if (shouldReply) sendAck(f.master, "SETMAXDG", "1");
    serialEventState("SETMAXDG");
    return;
  }

  // ------------------------------------------------------------------------
  // DGOFFSET (rechter Endschalter-Versatz in Deg01)
  //
  // Ziel:
  // - Rechter Endschalter kann mechanisch zu spaet ausloesen (z.B. erst bei 362,50deg).
  // - Logischer Arbeitsbereich bleibt 0..360,00deg.
  // - Offset wird SYMMETRISCH verteilt, damit "0" nicht in den linken Endschalter faehrt.
  //
  // Umsetzung:
  // - Offset wird als Deg01 gespeichert und in EncoderAxis als Skalierungs-Korrektur genutzt.
  // - Aenderung soll sofort wirken + persistent gespeichert werden.
  // ------------------------------------------------------------------------
  if (cmd == "GETDGOFFSET") {
    const int32_t v = safeI32(_cfg.dgOffsetDeg01, 0);
    if (shouldReply) sendAck(f.master, "GETDGOFFSET", formatDeg01ToString(v));
    return;
  }

  if (cmd == "SETDGOFFSET") {
    int32_t nv = parseDeg01NoClamp(f.params);
    if (nv < 0) nv = -nv;
    // Grobe Begrenzung: mehr als 90 Grad Offset macht praktisch keinen Sinn.
    if (nv > 9000) nv = 9000;

    persistPutI32("dgo", _cfg.dgOffsetDeg01, nv);

    if (shouldReply) sendAck(f.master, "SETDGOFFSET", "1");
    serialEventState("SETDGOFFSET");
    return;
  }

  // ------------------------------------------------------------------------
  // Homing PWM / Backoff / Return / Timeout
  // ------------------------------------------------------------------------
  if (cmd == "GETHOMEPWM") {
    const float v = (_cfg.homeFastPwmPercent) ? *_cfg.homeFastPwmPercent : 0.0f;
    if (shouldReply) sendAck(f.master, "GETHOMEPWM", formatFloatComma(v, 1));
    return;
  }

  if (cmd == "SETHOMEPWM") {
    const float v = clampPercent(parseFloatParam(f.params));
    persistPutFloat("hfp", _cfg.homeFastPwmPercent, v);
    applyHomingUpdate();
    if (shouldReply) sendAck(f.master, "SETHOMEPWM", "1");
    serialEventState("SETHOMEPWM");
    return;
  }

  // SEEK-MIN PWM (konstante PWM nach der Anfahr-Rampe Richtung END_MIN)
  // RS485:
  // - GETHOMESEEKPPWM
  // - SETHOMESEEKPPWM
  if (cmd == "GETHOMESEEKPPWM") {
    const float v = (_cfg.homeSeekMinPwmPercent) ? *_cfg.homeSeekMinPwmPercent : 0.0f;
    if (shouldReply) sendAck(f.master, "GETHOMESEEKPPWM", formatFloatComma(v, 1));
    return;
  }

  if (cmd == "SETHOMESEEKPPWM") {
    const float v = clampPercent(parseFloatParam(f.params));
    persistPutFloat("hsm", _cfg.homeSeekMinPwmPercent, v);
    applyHomingUpdate();
    if (shouldReply) sendAck(f.master, "SETHOMESEEKPPWM", "1");
    serialEventState("SETHOMESEEKPPWM");
    return;
  }

  if (cmd == "GETHOMEBACKOFF") {
    const float v = (_cfg.homeBackoff) ? *_cfg.homeBackoff : 0.0f;
    if (shouldReply) sendAck(f.master, "GETHOMEBACKOFF", formatFloatComma(v, 1));
    return;
  }

  if (cmd == "SETHOMEBACKOFF") {
    const float v = clampPercent(parseFloatParam(f.params));
    persistPutFloat("hbo", _cfg.homeBackoff, v);
    applyHomingUpdate();
    if (shouldReply) sendAck(f.master, "SETHOMEBACKOFF", "1");
    serialEventState("SETHOMEBACKOFF");
    return;
  }

  if (cmd == "GETHOMRETURN") {
    const bool v = (_cfg.homeReturnToZero) ? *_cfg.homeReturnToZero : false;
    if (shouldReply) sendAck(f.master, "GETHOMRETURN", v ? "1" : "0");
    return;
  }

  if (cmd == "SETHOMERETURN") {
    const bool v = parseBoolParam(f.params);
    persistPutBool("hrz", _cfg.homeReturnToZero, v);
    applyHomingUpdate();
    if (shouldReply) sendAck(f.master, "SETHOMERETURN", "1");
    serialEventState("SETHOMERETURN");
    return;
  }

  if (cmd == "GETHOMETIMEOUT") {
    const uint32_t v = safeU32(_cfg.homeTimeoutMs, 0);
    if (shouldReply) sendAck(f.master, "GETHOMETIMEOUT", String(v));
    return;
  }

  if (cmd == "SETHOMETIMEOUT") {
    const uint32_t v = parseU32Param(f.params);
    persistPutU32("hto", _cfg.homeTimeoutMs, v);
    applyHomingUpdate();
    if (shouldReply) sendAck(f.master, "SETHOMETIMEOUT", "1");
    serialEventState("SETHOMETIMEOUT");
    return;
  }

  // ------------------------------------------------------------------------
  // Positionsfahrt Timeout
  // ------------------------------------------------------------------------
  if (cmd == "GETPOSTIMEOUT") {
    const uint32_t v = safeU32(_cfg.posTimeoutMs, 0);
    if (shouldReply) sendAck(f.master, "GETPOSTIMEOUT", String(v));
    return;
  }

  if (cmd == "SETPOSTIMEOUT") {
    const uint32_t v = parseU32Param(f.params);
    persistPutU32("pto", _cfg.posTimeoutMs, v);
    if (shouldReply) sendAck(f.master, "SETPOSTIMEOUT", "1");
    serialEventState("SETPOSTIMEOUT");
    return;
  }

  // ------------------------------------------------------------------------
  // Hand-Speed (z.B. fuer manuelle Fahrten / UI)
  // ------------------------------------------------------------------------
  if (cmd == "GETHANDSPEED") {
    const float v = (_cfg.handSpeedPercent) ? *_cfg.handSpeedPercent : 0.0f;
    if (shouldReply) sendAck(f.master, "GETHANDSPEED", formatFloatComma(v, 1));
    return;
  }

  if (cmd == "SETHANDSPEED") {
    const float v = clampPercent(parseFloatParam(f.params));
    persistPutFloat("hsp", _cfg.handSpeedPercent, v);
    if (shouldReply) sendAck(f.master, "SETHANDSPEED", "1");
    serialEventState("SETHANDSPEED");
    return;
  }

  // ------------------------------------------------------------------------
  // Deadman / Command Timeout
  // ------------------------------------------------------------------------
  if (cmd == "GETDEADMAN") {
    const uint32_t v = safeU32(_cfg.cmdTimeoutMs, 0);
    if (shouldReply) sendAck(f.master, "GETDEADMAN", String(v));
    return;
  }

  if (cmd == "SETDEADMAN") {
    const uint32_t v = parseU32Param(f.params);
    persistPutU32("dman", _cfg.cmdTimeoutMs, v);
    applySafetyUpdate();
    if (shouldReply) sendAck(f.master, "SETDEADMAN", "1");
    serialEventState("SETDEADMAN");
    return;
  }

  // ------------------------------------------------------------------------
  // Stromueberwachung (IS)
  // ------------------------------------------------------------------------
  if (cmd == "GETIWARN") {
    const uint32_t v = safeU32(_cfg.isSoftWarnMv, 0);
    if (shouldReply) sendAck(f.master, "GETIWARN", String(v));
    return;
  }

  if (cmd == "SETIWARN") {
    const uint32_t v = parseU32Param(f.params);
    persistPutU32("iw", _cfg.isSoftWarnMv, v);
    applySafetyUpdate();
    if (shouldReply) sendAck(f.master, "SETIWARN", "1");
    serialEventState("SETIWARN");
    return;
  }

  if (cmd == "GETIMAX") {
    const uint32_t v = safeU32(_cfg.isHardStopMv, 0);
    if (shouldReply) sendAck(f.master, "GETIMAX", String(v));
    return;
  }

  if (cmd == "SETIMAX") {
    const uint32_t v = parseU32Param(f.params);
    persistPutU32("imax", _cfg.isHardStopMv, v);
    applySafetyUpdate();
    if (shouldReply) sendAck(f.master, "SETIMAX", "1");
    serialEventState("SETIMAX");
    return;
  }

  // ------------------------------------------------------------------------
  // Stromueberwachung: Details (Grace/Hold/Filter)
  // ------------------------------------------------------------------------
  // GETGRACEMS / SETISGRACEMS
  if (cmd == "GETGRACEMS") {
    const uint32_t v = safeU32(_cfg.isGraceMs, 0);
    if (shouldReply) sendAck(f.master, "GETGRACEMS", String(v));
    return;
  }

  if (cmd == "SETISGRACEMS") {
    const uint32_t v = parseU32Param(f.params);
    persistPutU32("igm", _cfg.isGraceMs, v);
    applySafetyUpdate();
    if (shouldReply) sendAck(f.master, "SETISGRACEMS", "1");
    serialEventState("SETISGRACEMS");
    return;
  }

  // GETISHOLDMS / SETISHOLDMS
  if (cmd == "GETISHOLDMS") {
    const uint32_t v = safeU32(_cfg.isHardHoldMs, 0);
    if (shouldReply) sendAck(f.master, "GETISHOLDMS", String(v));
    return;
  }

  if (cmd == "SETISHOLDMS") {
    const uint32_t v = parseU32Param(f.params);
    persistPutU32("ihm", _cfg.isHardHoldMs, v);
    applySafetyUpdate();
    if (shouldReply) sendAck(f.master, "SETISHOLDMS", "1");
    serialEventState("SETISHOLDMS");
    return;
  }

  // GETFILTERLEN / SETISFILTERLEN
  if (cmd == "GETFILTERLEN") {
    const uint8_t v = safeU8(_cfg.isFilterLen, 0);
    if (shouldReply) sendAck(f.master, "GETFILTERLEN", String((int)v));
    return;
  }

  if (cmd == "SETISFILTERLEN") {
    uint8_t v = parseU8Param(f.params);
    if (v < 1) v = 1;
    if (v > 32) v = 32;
    persistPutU8("ifl", _cfg.isFilterLen, v);
    applySafetyUpdate();
    if (shouldReply) sendAck(f.master, "SETISFILTERLEN", "1");
    serialEventState("SETISFILTERLEN");
    return;
  }

  // ------------------------------------------------------------------------
  // Arrival-Toleranz (deg01)
  // ------------------------------------------------------------------------
  if (cmd == "GETARRTOL") {
    const int32_t v = (_cfg.arriveTolDeg01) ? *_cfg.arriveTolDeg01 : 0;
    if (shouldReply) sendAck(f.master, "GETARRTOL", formatDeg01ToString(v));
    return;
  }

  if (cmd == "SETARRTOL") {
    int32_t v = parseDeg01NoClamp(f.params);
    if (v < 0) v = 0;
    persistPutI32("atol", _cfg.arriveTolDeg01, v);
    if (shouldReply) sendAck(f.master, "SETARRTOL", "1");
    serialEventState("SETARRTOL");
    return;
  }

  // ------------------------------------------------------------------------
  // Rampenlaenge (Grad): Regler; Live-Bins: max(ramp, SETCALIGNDG). GETACCBINS: SETRAPDG.
  // ------------------------------------------------------------------------
  if (cmd == "GETRAMP") {
    const float v = (_cfg.rampDistDeg) ? *_cfg.rampDistDeg : 0.0f;
    if (shouldReply) sendAck(f.master, "GETRAMP", formatFloatComma(v, 1));
    return;
  }

  if (cmd == "SETRAMP") {
    float v = parseFloatParam(f.params);
    if (v < 0.0f) v = 0.0f;
    persistPutFloat("ramp", _cfg.rampDistDeg, v);
    if (shouldReply) sendAck(f.master, "SETRAMP", "1");
    serialEventState("SETRAMP");
    return;
  }

  // ------------------------------------------------------------------------
  // Min PWM
  // ------------------------------------------------------------------------
  if (cmd == "GETMINPWM") {
    const float v = (_cfg.minPwm) ? *_cfg.minPwm : 0.0f;
    if (shouldReply) sendAck(f.master, "GETMINPWM", formatFloatComma(v, 1));
    return;
  }

  if (cmd == "SETMINPWM") {
    const float v = clampPercent(parseFloatParam(f.params));
    persistPutFloat("minp", _cfg.minPwm, v);
    if (shouldReply) sendAck(f.master, "SETMINPWM", "1");
    serialEventState("SETMINPWM");
    return;
  }

  // ------------------------------------------------------------------------
  // Stall Timeout
  // ------------------------------------------------------------------------
  if (cmd == "GETSTALLTIMEOUT") {
    const uint32_t v = safeU32(_cfg.stallTimeoutMs, 0);
    if (shouldReply) sendAck(f.master, "GETSTALLTIMEOUT", String(v));
    return;
  }

  if (cmd == "SETSTALLTIMEOUT") {
    const uint32_t v = parseU32Param(f.params);
    persistPutU32("stto", _cfg.stallTimeoutMs, v);
    applySafetyUpdate();
    if (shouldReply) sendAck(f.master, "SETSTALLTIMEOUT", "1");
    serialEventState("SETSTALLTIMEOUT");
    return;
  }

  // ------------------------------------------------------------------------
  // Stall-Erkennung: Enable / Arm-PWM / Min-Counts
  // ------------------------------------------------------------------------
  // GETSTALLEN / SETSTALLEN
  if (cmd == "GETSTALLEN") {
    const bool v = safeBool(_cfg.stallMonitorEnabled, false);
    if (shouldReply) sendAck(f.master, "GETSTALLEN", v ? "1" : "0");
    return;
  }

  if (cmd == "SETSTALLEN") {
    const bool v = parseBoolParam(f.params);
    persistPutBool("sten", _cfg.stallMonitorEnabled, v);
    applySafetyUpdate();
    if (shouldReply) sendAck(f.master, "SETSTALLEN", "1");
    serialEventState("SETSTALLEN");
    return;
  }

  // GETMINSTALLPWM / SETMINSTALLPWM
  if (cmd == "GETMINSTALLPWM") {
    const float v = (_cfg.minStallPwm) ? *_cfg.minStallPwm : 0.0f;
    if (shouldReply) sendAck(f.master, "GETMINSTALLPWM", formatFloatComma(v, 1));
    return;
  }

  if (cmd == "SETMINSTALLPWM") {
    const float v = clampPercent(parseFloatParam(f.params));
    persistPutFloat("msp", _cfg.minStallPwm, v);
    applySafetyUpdate();
    if (shouldReply) sendAck(f.master, "SETMINSTALLPWM", "1");
    serialEventState("SETMINSTALLPWM");
    return;
  }

  // GETSTALLMINCOUNTS / SETSTALLMINCOUNTS
  if (cmd == "GETSTALLMINCOUNTS") {
    const uint32_t v = safeU32(_cfg.stallMinCounts, 0);
    if (shouldReply) sendAck(f.master, "GETSTALLMINCOUNTS", String(v));
    return;
  }

  if (cmd == "SETSTALLMINCOUNTS") {
    const uint32_t v = parseU32Param(f.params);
    persistPutU32("smc", _cfg.stallMinCounts, v);
    applySafetyUpdate();
    if (shouldReply) sendAck(f.master, "SETSTALLMINCOUNTS", "1");
    serialEventState("SETSTALLMINCOUNTS");
    return;
  }

  // ------------------------------------------------------------------------
  // EncoderType / Encoder-ExpectedCounts (erfordern Neustart)
  // ------------------------------------------------------------------------
  // GETENCTYPE / SETENCTYPE
  // Werte:
  // - 1 = ENCTYPE_MOTOR_AXIS
  // - 2 = ENCTYPE_RING_OUTPUT
  if (cmd == "GETENCTYPE") {
    uint8_t v = safeU8(_cfg.encTypeU8, 2);
    if (v != 1 && v != 2) v = 2;
    if (shouldReply) sendAck(f.master, "GETENCTYPE", String((int)v));
    return;
  }

  if (cmd == "SETENCTYPE") {
    const uint8_t nv = parseU8Param(f.params);
    if (nv != 1 && nv != 2) {
      if (shouldReply) sendNak(f.master, "SETENCTYPE", "BADVAL");
      return;
    }

    const uint8_t old = safeU8(_cfg.encTypeU8, 2);
    persistPutU8("ect", _cfg.encTypeU8, nv);

    if (shouldReply) sendAck(f.master, "SETENCTYPE", "1");
    serialEventState("SETENCTYPE");

    // Nur dann reboot, wenn sich wirklich etwas aendert.
    if (old != nv) {
      requestRestart(250);
    }
    return;
  }

  // GETENCCRI / SETENCCRI (ExpectedCounts Ring)
  if (cmd == "GETENCCRI") {
    const int32_t v = safeI32(_cfg.homeExpectedCountsRing, 0);
    if (shouldReply) sendAck(f.master, "GETENCCRI", String((long)v));
    return;
  }

  if (cmd == "SETENCCRI") {
    const int32_t nv = parseI32Param(f.params);
    if (nv <= 0) {
      if (shouldReply) sendNak(f.master, "SETENCCRI", "BADVAL");
      return;
    }
    const int32_t old = safeI32(_cfg.homeExpectedCountsRing, 0);
    persistPutI32("ecr", _cfg.homeExpectedCountsRing, nv);
    if (shouldReply) sendAck(f.master, "SETENCCRI", "1");
    serialEventState("SETENCCRI");
    if (old != nv) {
      requestRestart(250);
    }
    return;
  }

  // GETENCCAX / SETENCCAX (ExpectedCounts Motorachse)
  if (cmd == "GETENCCAX") {
    const int32_t v = safeI32(_cfg.homeExpectedCountsMotor, 0);
    if (shouldReply) sendAck(f.master, "GETENCCAX", String((long)v));
    return;
  }

  if (cmd == "SETENCCAX") {
    const int32_t nv = parseI32Param(f.params);
    if (nv <= 0) {
      if (shouldReply) sendNak(f.master, "SETENCCAX", "BADVAL");
      return;
    }
    const int32_t old = safeI32(_cfg.homeExpectedCountsMotor, 0);
    persistPutI32("ecm", _cfg.homeExpectedCountsMotor, nv);
    if (shouldReply) sendAck(f.master, "SETENCCAX", "1");
    serialEventState("SETENCCAX");
    if (old != nv) {
      requestRestart(250);
    }
    return;
  }

  // ------------------------------------------------------------------------
  // PWM-Max persistent + runtime
  // ------------------------------------------------------------------------
  if (cmd == "GETMAXPWM") {
    const float v = (_cfg.pwmMaxAbsStored) ? *_cfg.pwmMaxAbsStored : 0.0f;
    if (shouldReply) sendAck(f.master, "GETMAXPWM", formatFloatComma(v, 1));
    return;
  }

  if (cmd == "SETMAXPWM") {
    const float v = clampPercent(parseFloatParam(f.params));

    // persistent speichern
    persistPutFloat("maxp", _cfg.pwmMaxAbsStored, v);

    // runtime ebenfalls sofort uebernehmen
    if (_cfg.pwmMaxAbsRuntime) *_cfg.pwmMaxAbsRuntime = v;

    if (shouldReply) sendAck(f.master, "SETMAXPWM", "1");
    serialEventState("SETMAXPWM");
    return;
  }

  // ------------------------------------------------------------------------
// PWM runtime-only (nicht speichern)
// ------------------------------------------------------------------------
// GETPWM: aktueller Laufzeitwert fuer PWM-Max (kann ueber SETPWM geaendert werden).
// Hinweis: Dieser Wert wird NICHT in Preferences gespeichert und geht nach Neustart verloren.
if (cmd == "GETPWM") {
  float v = 0.0f;
  if (_cfg.pwmMaxAbsRuntime) {
    v = *_cfg.pwmMaxAbsRuntime;
  } else if (_cfg.pwmMaxAbsStored) {
    // Fallback: falls keine Runtime-Variable konfiguriert wurde.
    v = *_cfg.pwmMaxAbsStored;
  }
  if (shouldReply) sendAck(f.master, "GETPWM", formatFloatComma(v, 1));
  serialEventState("GETPWM");
  return;
}

// SETPWM: setzt nur den Laufzeitwert (ohne Speichern).
if (cmd == "SETPWM") {
  const float v = clampPercent(parseFloatParam(f.params));

  if (!_cfg.pwmMaxAbsRuntime) {
    if (shouldReply) sendNak(f.master, "SETPWM", "NORUNTIME");
    serialEventState("SETPWM");
    return;
  }

  // Nur Laufzeitwert, nicht speichern (geht nach Neustart verloren)
  *_cfg.pwmMaxAbsRuntime = v;

  if (shouldReply) sendAck(f.master, "SETPWM", formatFloatComma(v, 1));
  serialEventState("SETPWM");
  return;
}

  // ------------------------------------------------------------------------
  // Unbekannt
  // ------------------------------------------------------------------------
  if (shouldReply) sendNak(f.master, cmd, "NOTIMPL");
}
