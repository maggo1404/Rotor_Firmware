#include "Rs485Proto.h"

static bool parseU8Strict(const String& s, uint8_t& out) {
  String t = s;
  t.trim();
  if (t.length() == 0) return false;

  for (uint16_t i = 0; i < t.length(); i++) {
    char ch = t[i];
    if (!(ch >= '0' && ch <= '9')) return false;
  }

  long v = t.toInt();
  if (v < 0 || v > 255) return false;

  out = (uint8_t)v;
  return true;
}

void Rs485Proto::begin(HardwareSerial& uart,
                       uint32_t baud,
                       gpio_num_t dirPin,
                       int rxPin,
                       int txPin,
                       uint8_t ownSlaveId,
                       uint16_t tPreUs,
                       uint16_t tPostUs,
                       size_t rxBufferBytes) {
  _uart = &uart;
  _dirPin = dirPin;
  _ownSlaveId = ownSlaveId;

  _tPreUs  = tPreUs;
  _tPostUs = tPostUs;
  _rxBufferBytes = (rxBufferBytes >= 256) ? rxBufferBytes : 256;

  // Wichtig:
  // - RX-Puffer VOR begin() vergroessern, damit auch laengere Bursts sauber im UART-Treiber landen.
  // - Rueckgabewert ignorieren wir absichtlich: auf manchen Core-Versionen ist die API vorhanden,
  //   liefert aber nur bool ohne weitere Diagnose. Der Wunschwert bleibt hier trotzdem dokumentiert.
  _uart->setRxBufferSize(_rxBufferBytes);
  _uart->begin(baud, SERIAL_8N1, rxPin, txPin);

  pinMode((int)_dirPin, OUTPUT);
  digitalWrite((int)_dirPin, LOW); // RX default

  _rxOk.store(0);
  _rxBad.store(0);
  _rxQueueDrops.store(0);
  _lastBadRaw.reserve(240);
  _lastBadRaw = "";

  if (_ioMutex == nullptr) {
    _ioMutex = xSemaphoreCreateMutex();
  }

  if (_rxQueue == nullptr) {
    _rxQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(RxQueuedLine));
  }

  // RX-Task nur einmal starten.
  // Prio relativ hoch: UART bytes nicht verlieren, Mutex mit TX koordinieren.
  if ((_rxTaskHandle == nullptr) && (_rxQueue != nullptr)) {
    xTaskCreatePinnedToCore(
      Rs485Proto::rxTaskTrampoline,
      "rs485_rx_task",
      4096,
      this,
      5,
      &_rxTaskHandle,
      0);
  }
}

void Rs485Proto::setOwnSlaveId(uint8_t ownSlaveId) {
  _ownSlaveId = ownSlaveId;
}

void Rs485Proto::setTxMode(bool tx) {
  digitalWrite((int)_dirPin, tx ? HIGH : LOW);
}

bool Rs485Proto::readLine(String& outLine) {
  // Holt ein komplettes Roh-Frame aus der RX-Queue.
  // Die eigentliche UART-Leselogik laeuft im Hintergrund-Task.
  if (_rxQueue == nullptr) return false;

  RxQueuedLine raw{};
  if (xQueueReceive(_rxQueue, &raw, 0) != pdTRUE) {
    return false;
  }

  outLine = String(raw.text);
  return true;
}

void Rs485Proto::rxTaskTrampoline(void* arg) {
  Rs485Proto* self = static_cast<Rs485Proto*>(arg);
  if (self != nullptr) {
    self->rxTaskLoop();
  }
  vTaskDelete(nullptr);
}

void Rs485Proto::rxTaskLoop() {
  // Der Task liest ausschliesslich von Serial1,
  // erkennt komplette #...$-Frames und legt sie in die Queue.
  // Dadurch ist der RS485-Empfang nicht mehr vom loop()-Takt abhaengig.
  char acc[RX_RAW_FRAME_MAX];
  size_t accLen = 0;
  acc[0] = '\0';

  for (;;) {
    bool didWork = false;

    if ((_uart != nullptr) && (_ioMutex != nullptr)) {
      // Nicht mit kurzem Timeout abbrechen: waehrend sendFrame() den Mutex haelt,
      // muss der Empfang danach zuverlaessig weiterlaufen — sonst Luecken/FIFO-Ueberlauf.
      if (xSemaphoreTake(_ioMutex, portMAX_DELAY) == pdTRUE) {
        while (_uart->available() > 0) {
          int rv = _uart->read();
          if (rv < 0) break;
          processRxByte((char)rv, acc, accLen);
          didWork = true;
        }
        xSemaphoreGive(_ioMutex);
      }
    }

    // Kein Busy-Wait:
    // Wenn nichts zu lesen war, lassen wir dem System kurz Luft.
    if (!didWork) {
      vTaskDelay(pdMS_TO_TICKS(1));
    } else {
      taskYIELD();
    }
  }
}

void Rs485Proto::processRxByte(char c, char* acc, size_t& accLen) {
  const uint8_t uc = (uint8_t)c;

  // Resync:
  // Ein neues '#' startet IMMER ein neues Frame.
  // Damit fangen wir uns nach Fragmenten oder Stoerbytes schnell wieder ein.
  if (c == '#') {
    accLen = 0;
    acc[accLen++] = '#';
    acc[accLen] = '\0';
    return;
  }

  // Solange noch kein Start gesehen wurde, ignorieren wir alles.
  if (accLen == 0) return;

  // Frame-Ende -> komplettes Roh-Frame in die Queue schieben.
  if (c == '$') {
    if (accLen < (RX_RAW_FRAME_MAX - 1)) {
      acc[accLen++] = '$';
      acc[accLen] = '\0';
      enqueueRawLine(acc);
    } else {
      _rxBad.fetch_add(1, std::memory_order_relaxed);
    }

    accLen = 0;
    acc[0] = '\0';
    return;
  }

  // Noise-Filter: nur druckbare ASCII-Zeichen sowie CR/LF/TAB zulassen.
  const bool printable = (uc >= 0x20 && uc <= 0x7E);
  const bool whitespace = (c == '\r' || c == '\n' || c == '\t');
  if (!(printable || whitespace)) {
    return;
  }

  // Schutz gegen runaway / unvollstaendige Monster-Frames.
  if (accLen >= (RX_RAW_FRAME_MAX - 2)) {
    _rxBad.fetch_add(1, std::memory_order_relaxed);
    accLen = 0;
    acc[0] = '\0';
    return;
  }

  acc[accLen++] = c;
  acc[accLen] = '\0';
}

void Rs485Proto::enqueueRawLine(const char* line) {
  if ((_rxQueue == nullptr) || (line == nullptr)) return;

  RxQueuedLine item{};
  strlcpy(item.text, line, sizeof(item.text));

  // Wenn die Queue voll ist, verwerfen wir das aelteste Frame und behalten das neue.
  // Fuer den Nutzer ist das in der Praxis meist besser, weil aktuelle Kommandos wichtiger sind.
  if (xQueueSend(_rxQueue, &item, 0) == pdTRUE) {
    return;
  }

  RxQueuedLine dummy{};
  (void)xQueueReceive(_rxQueue, &dummy, 0);

  if (xQueueSend(_rxQueue, &item, 0) != pdTRUE) {
    _rxQueueDrops.fetch_add(1, std::memory_order_relaxed);
  }
}

bool Rs485Proto::parseDecimalScaled100(const String& s, int32_t& outScaled) const {
  String t = s;
  t.trim();
  if (t.length() == 0) return false;

  // Vorzeichen
  bool neg = false;
  if (t[0] == '-') {
    neg = true;
    t = t.substring(1);
  } else if (t[0] == '+') {
    t = t.substring(1);
  }

  t.trim();
  if (t.length() == 0) return false;

  // Dezimaltrenner vereinheitlichen
  t.replace(",", ".");

  int dot = t.indexOf('.');
  String a = (dot >= 0) ? t.substring(0, dot) : t;
  String b = (dot >= 0) ? t.substring(dot + 1) : "";

  a.trim();
  b.trim();

  if (a.length() == 0) a = "0";

  // Integerteil pruefen
  for (uint16_t i = 0; i < a.length(); i++) {
    char ch = a[i];
    if (!(ch >= '0' && ch <= '9')) return false;
  }

  // Fraction pruefen (nur Ziffern)
  for (uint16_t i = 0; i < b.length(); i++) {
    char ch = b[i];
    if (!(ch >= '0' && ch <= '9')) return false;
  }

  long intPart = a.toInt();

  // Fraction auf 2 Stellen normalisieren (0,01)
  long frac = 0;
  if (b.length() == 0) {
    frac = 0;
  } else if (b.length() == 1) {
    frac = (b.toInt() * 10);
  } else {
    // 2 oder mehr Stellen -> wir nehmen die ersten 2 (kein Runden, deterministisch)
    String b2 = b.substring(0, 2);
    frac = b2.toInt();
  }

  long scaled = intPart * 100L + frac;
  if (neg) scaled = -scaled;

  outScaled = (int32_t)scaled;
  return true;
}

int32_t Rs485Proto::extractValueScaled100(const String& params) const {
  String p = params;
  p.trim();
  if (p.length() == 0) return 0;

  // Letzten Token nach ':' nehmen (damit "L:25" -> 25)
  int lastColon = p.lastIndexOf(':');
  String token = (lastColon >= 0) ? p.substring(lastColon + 1) : p;
  token.trim();

  // Wenn params eine Liste enthaelt (z.B. "1;2;3"),
  // nehmen wir fuer die Checksumme das letzte Listenelement.
  // Damit bleibt die Checksumme auch bei Mehrfach-Warnungen stabil/pruefbar.
  // (Leer-Elemente am Ende werden robust entfernt.)
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
  if (!parseDecimalScaled100(token, vScaled)) return 0;

  // Fuer die Checksumme benutzen wir den Betrag (damit z.B. "-12" nicht komisch wird)
  if (vScaled < 0) vScaled = -vScaled;
  return vScaled;
}

int32_t Rs485Proto::computeChecksumScaled100(uint8_t src, uint8_t dst, const String& params) const {
  // chk_scaled = (src + dst) * 100 + wert_scaled
  int32_t v = extractValueScaled100(params);
  int32_t sum = ((int32_t)src + (int32_t)dst) * 100 + v;
  return sum;
}

String Rs485Proto::formatScaled100(int32_t scaled) const {
  // Ausgabe: ohne Nachkommastellen wenn .00
  // ansonsten immer mit 2 Nachkommastellen (Komma als Trenner)
  bool neg = false;
  if (scaled < 0) {
    neg = true;
    scaled = -scaled;
  }

  int32_t ip = scaled / 100;
  int32_t fp = scaled % 100;

  if (fp == 0) {
    return neg ? String("-") + String(ip) : String(ip);
  }

  String s;
  s.reserve(16);
  if (neg) s += "-";
  s += String(ip);
  s += ",";
  if (fp < 10) s += "0";
  s += String(fp);
  return s;
}

Rs485Frame Rs485Proto::parseFrame(const String& raw) {
  Rs485Frame f;

  if (raw.length() < 5) return f;
  if (raw.charAt(0) != '#') return f;
  if (raw.charAt(raw.length() - 1) != '$') return f;

  String body = raw.substring(1, raw.length() - 1);
  body.trim();

  // Format: src:dst:cmd:params:chk
  // params darf ':' enthalten -> chk immer nach dem letzten ':'
  int p1 = body.indexOf(':');
  if (p1 < 0) return f;

  int p2 = body.indexOf(':', p1 + 1);
  if (p2 < 0) return f;

  int p3 = body.indexOf(':', p2 + 1);
  if (p3 < 0) return f;

  int pLast = body.lastIndexOf(':');
  if (pLast <= p3) return f;

  String sMaster = body.substring(0, p1);
  String sSlave  = body.substring(p1 + 1, p2);
  String cmd     = body.substring(p2 + 1, p3);
  String params  = body.substring(p3 + 1, pLast);
  String sChk    = body.substring(pLast + 1);

  uint8_t m = 0;
  uint8_t s = 0;

  if (!parseU8Strict(sMaster, m)) return f;
  if (!parseU8Strict(sSlave, s)) return f;

  cmd.trim();
  params.trim();
  sChk.trim();

  if (cmd.length() == 0) return f;

  // Checksumme (skalierte Zahl) parsen
  int32_t chkRxScaled = 0;
  if (!parseDecimalScaled100(sChk, chkRxScaled)) return f;
  if (chkRxScaled < 0) chkRxScaled = -chkRxScaled;

  // Felder setzen (auch bei invalid, damit Logging klappt)
  f.master = m;
  f.slave  = s;
  f.cmd    = cmd;
  f.params = params;

  int32_t chkExpScaled = computeChecksumScaled100(m, s, params);

  if (chkExpScaled != chkRxScaled) {
    f.valid = false;
    return f;
  }

  f.valid = true;
  return f;
}

bool Rs485Proto::poll(Rs485Frame& out) {
  if (_rxQueue == nullptr) return false;

  String line;
  if (!readLine(line)) return false;

  Rs485Frame f = parseFrame(line);

  if (!f.valid) {
    _rxBad.fetch_add(1, std::memory_order_relaxed);
    _lastBadRaw = line;
  } else {
    _rxOk.fetch_add(1, std::memory_order_relaxed);
  }

  out = f;
  return true;
}

void Rs485Proto::sendFrame(uint8_t src, uint8_t dst, const String& cmd, const String& params) {
  if ((_uart == nullptr) || (_ioMutex == nullptr)) return;

  int32_t chkScaled = computeChecksumScaled100(src, dst, params);

  String frame;
  frame.reserve(200);

  frame += "#";
  frame += String(src);
  frame += ":";
  frame += String(dst);
  frame += ":";
  frame += cmd;
  frame += ":";
  frame += params;
  frame += ":";
  frame += formatScaled100(chkScaled);
  frame += "$";

  // Warten bis RX-Task UART freigegeben hat — kein stilles „kein ACK“ nach 20 ms.
  if (xSemaphoreTake(_ioMutex, portMAX_DELAY) != pdTRUE) {
    return;
  }

  setTxMode(true);
  delayMicroseconds(_tPreUs);

  _uart->print(frame);
  _uart->flush();

  delayMicroseconds(_tPostUs);
  setTxMode(false);
  xSemaphoreGive(_ioMutex);
  taskYIELD();
}

void Rs485Proto::sendPayloadFrame(uint8_t src, uint8_t dst, const String& payload) {
  // Kompaktes Frame ohne CMD-Feld (siehe Rs485Proto.h)
  // Format: #src:dst:payload:chk$
  // Wichtig: Die Checksumme wird wie gewohnt nur aus src/dst und payload gebildet.
  if ((_uart == nullptr) || (_ioMutex == nullptr)) return;

  int32_t chkScaled = computeChecksumScaled100(src, dst, payload);

  String frame;
  frame.reserve(200);

  frame += "#";
  frame += String(src);
  frame += ":";
  frame += String(dst);
  frame += ":";
  frame += payload;
  frame += ":";
  frame += formatScaled100(chkScaled);
  frame += "$";

  if (xSemaphoreTake(_ioMutex, portMAX_DELAY) != pdTRUE) {
    return;
  }

  setTxMode(true);
  delayMicroseconds(_tPreUs);

  _uart->print(frame);
  _uart->flush();

  delayMicroseconds(_tPostUs);
  setTxMode(false);
  xSemaphoreGive(_ioMutex);
  taskYIELD();
}
