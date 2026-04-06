#pragma once

#include <atomic>

#include <Arduino.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// Geparstes RS485-Frame.
struct Rs485Frame {
  uint8_t master = 0;
  uint8_t slave  = 0;
  String  cmd;
  String  params;
  bool    valid  = false;
};

class Rs485Proto {
public:
  Rs485Proto() = default;

  void begin(HardwareSerial& uart,
             uint32_t baud,
             gpio_num_t dirPin,
             int rxPin,
             int txPin,
             uint8_t ownSlaveId,
             uint16_t tPreUs = 150,
             uint16_t tPostUs = 150,
             size_t rxBufferBytes = 2048);

  // Eigene Slave-ID zur Laufzeit aendern (z.B. nach SETID)
  void setOwnSlaveId(uint8_t ownSlaveId);

  // Frame empfangen (nicht-blockierend)
  // Wichtig:
  // - Liest NICHT mehr direkt aus der UART.
  // - Holt fertige Frames aus der RX-Queue, die vom Hintergrund-Task gefuellt wird.
  bool poll(Rs485Frame& out);

  // Frame senden
  void sendFrame(uint8_t src, uint8_t dst, const String& cmd, const String& params);

  // Kompaktes Frame senden (ohne CMD-Feld):
  // Format: #src:dst:payload:chk$
  // Nutzen wir u.a. fuer GETWARN/GETERR (schnell parsen auf Master-Seite).
  void sendPayloadFrame(uint8_t src, uint8_t dst, const String& payload);

  // Diagnose
  uint32_t getRxFramesOk() const { return _rxOk.load(std::memory_order_relaxed); }
  uint32_t getRxFramesBad() const { return _rxBad.load(std::memory_order_relaxed); }
  uint32_t getRxQueueDrops() const { return _rxQueueDrops.load(std::memory_order_relaxed); }
  String getLastBadRaw() const { return _lastBadRaw; }

  // Zahl aus scaled100 (0,01) fuer RS485-Frames formatieren.
  // Wichtig: nutzt Dezimaltrenner ',' und gibt das Vorzeichen mit aus.
  String formatScaled100(int32_t scaled) const;

private:
  // Roh-Frame fuer die RX-Queue.
  // Wir nutzen absichtlich KEIN String im Queue-Element,
  // damit der FreeRTOS-Queue-Kopiepfad sauber und deterministisch bleibt.
  static constexpr size_t RX_RAW_FRAME_MAX = 224;
  // Tiefe RX-Queue: bei Bus-Flut (Master-Polling + Wetter + GETPOS) verhindert
  // zu kleine Queues verworfene Telegramme (Master sieht dann „kein ACK“).
  static constexpr UBaseType_t RX_QUEUE_LENGTH = 128;

  struct RxQueuedLine {
    char text[RX_RAW_FRAME_MAX];
  };

  void setTxMode(bool tx);
  bool readLine(String& outLine);
  Rs485Frame parseFrame(const String& raw);

  // Hintergrund-Task fuer Serial1-RX.
  static void rxTaskTrampoline(void* arg);
  void rxTaskLoop();
  void processRxByte(char c, char* acc, size_t& accLen);
  void enqueueRawLine(const char* line);

  // Checksumme (Fixed-Point 0,01)
  int32_t computeChecksumScaled100(uint8_t src, uint8_t dst, const String& params) const;
  bool parseDecimalScaled100(const String& s, int32_t& outScaled) const;
  int32_t extractValueScaled100(const String& params) const;

private:
  HardwareSerial* _uart = nullptr;
  gpio_num_t _dirPin = GPIO_NUM_NC;
  uint8_t _ownSlaveId = 0;

  uint16_t _tPreUs  = 150;
  uint16_t _tPostUs = 150;

  // Schutz fuer die gemeinsame UART / DE-RE-Umschaltung.
  // - RX-Task liest darueber Serial1.
  // - sendFrame()/sendPayloadFrame() senden darueber Antworten.
  // Dadurch greift nie gleichzeitig ein Sende- und ein Lesepfad auf dieselbe UART zu.
  SemaphoreHandle_t _ioMutex = nullptr;

  // RX-Task + Queue fuer fertig erkannte Frames.
  QueueHandle_t _rxQueue = nullptr;
  TaskHandle_t _rxTaskHandle = nullptr;
  size_t _rxBufferBytes = 2048;

  // Diagnosezaehler (RX-Task + poll()/loop — atomic statt volatile++/deprecated).
  std::atomic<uint32_t> _rxOk{0};
  std::atomic<uint32_t> _rxBad{0};
  std::atomic<uint32_t> _rxQueueDrops{0};
  String   _lastBadRaw;
};
