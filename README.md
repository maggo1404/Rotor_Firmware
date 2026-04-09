# Rotor Firmware

Firmware für einen motorisierten Antennenrotor basierend auf ESP32-S3.

## Überblick

Dieses Projekt steuert einen motorisierten Rotor für Richtantennen (z.B. für Amateurfunk, Satellitenkommunikation oder Wetterstationen). Die Firmware läuft auf einem ESP32-S3 und bietet präzise Positionsregelung, Homing-Funktionalität und RS485-Kommunikation.

### Hauptfunktionen

- **Präzise Positionsregelung** mit Encoder-Rückführung (Motor- oder Ring-Encoder)
- **Automatisches Homing** mit Endschalter-Erkennung und Backlash-Kompensation
- **RS485-Kommunikation** für Fernsteuerung und Statusabfragen
- **Stromüberwachung** (IS-Messung) zur Blockadeerkennung
- **Temperaturüberwachung** (DS18B20 Sensoren)
- **Windmessung** über Anemometer und optionalen RS485-Windsensor
- **Sicherheitsfunktionen**: Stall-Erkennung, Endschalter-Überwachung, Deadman/Keepalive

## Hardware

- **MCU**: ESP32-S3 (8MB Flash, 8MB PSRAM)
- **Motorsteuerung**: H-Brücke mit PWM-Ansteuerung
- **Encoder**: Quadratur-Encoder (optional mit Z-Index)
- **Kommunikation**: RS485 (Half-Duplex)
- **Sensoren**: DS18B20 (Temperatur), Anemometer (Windgeschwindigkeit)

## Bauen

### Voraussetzungen

- [PlatformIO](https://platformio.org/) (VS Code Extension oder CLI)
- USB-Kabel für ESP32-S3

### Kompilieren & Flashen

```bash
# Build
pio run

# Build & Upload
pio run --target upload

# Serial Monitor
pio device monitor
```

### Umgebung

Standard-Umgebung: `esp32-s3-n8r8`

```bash
# Explizit für ESP32-S3 bauen
pio run --environment esp32-s3-n8r8
```

## Konfiguration

Die Firmware speichert Konfigurationswerte persistent im NVS (Non-Volatile Storage). Wichtige Parameter:

| Parameter | Beschreibung | Standard |
|-----------|--------------|----------|
| `slaveId` | RS485 Slave-ID | 20 |
| `axisMinDeg01` | Minimale Achsposition (0.01°) | 0 |
| `axisMaxDeg01` | Maximale Achsposition (0.01°) | 36000 (360°) |
| `homeFastPwmPercent` | Homing-Geschwindigkeit (%) | 100 |
| `minPwm` | Mindest-PWM für Bewegung (%) | 25 |
| `stallTimeoutMs` | Timeout für Stall-Erkennung (ms) | 2000 |

### Werksreset

Beide Handspeed-Taster beim Booten gedrückt halten → NVS wird gelöscht und das Gerät startet mit Standardwerten neu.

## RS485 Kommandos

Die Kommunikation erfolgt über RS485 mit konfigurierbarer Slave-ID.

### Grundlegende Kommandos

| Kommando | Beschreibung |
|----------|--------------|
| `GETPOSDG` | Aktuelle Position in 0.01° abfragen |
| `SETPOSDG:<wert>` | Zielposition in 0.01° setzen |
| `GETERR` | Aktuellen Fehlercode abfragen |
| `SETREF` | Fehler quittieren / Referenz setzen |
| `HOME` | Homing-Sequenz starten |
| `STOP` | Bewegung stoppen |

### Konfigurations-Kommandos

| Kommando | Beschreibung |
|----------|--------------|
| `SETID:<id>` | Slave-ID setzen (1-247) |
| `SETPWM:<prozent>` | Maximale PWM setzen (0-100%) |
| `SETMINPWM:<prozent>` | Mindest-PWM setzen |
| `SETSTALLTO:<ms>` | Stall-Timeout setzen |
| `GETTEMPA` | Umgebungstemperatur abfragen |
| `GETTEMPM` | Motortemperatur abfragen |
| `GETIS` | Strommesswerte abfragen |

## Firmware herunterladen

Fertige Firmware-Binaries werden automatisch durch GitHub Actions gebaut und als Artefakte gespeichert.

### Schritte zum Download

1. Gehe zu **[Actions → PlatformIO Build](https://github.com/maggo1404/Rotor_Firmware/actions/workflows/platformio-build.yml)**
2. Wähle den neuesten erfolgreichen Workflow-Run
3. Scrolle zu **Artifacts** und lade `firmware-bin` herunter
4. Entpacke das ZIP-Archiv

### Dateien im Artifact

| Datei | Beschreibung |
|-------|--------------|
| `firmware.bin` | Haupt-Firmware |
| `bootloader.bin` | Bootloader |
| `partitions.bin` | Partitionstabelle |
| `firmware.elf` | Debug-Symbole |

### Flashen der heruntergeladenen Firmware

Mit **esptool.py** (Python erforderlich):

```bash
# Alle Partitionen flashen
esptool.py --chip esp32-s3 --port COM15 write_flash \
  0x0000 bootloader.bin \
  0x8000 partitions.bin \
  0x10000 firmware.bin
```

Mit **PlatformIO** (nur `firmware.bin`):

```bash
# Nur die App-Firmware flashen (Bootloader/Partitionen bleiben erhalten)
pio run --target upload --upload-port COM15
```

## Projektstruktur

```
Rotor_Firmware/
├── src/
│   └── main.cpp              # Hauptanwendung
├── lib/                      # Lokale Bibliotheken
│   ├── HalBoard/             # Hardware-Abstraktion
│   ├── MotorMcpwm/           # Motorsteuerung
│   ├── EncoderAxis/          # Encoder-Verarbeitung
│   ├── Rs485Proto/           # RS485-Protokoll
│   ├── MotionController/     # Positionsregelung
│   ├── HomingController/     # Homing-Logik
│   ├── SafetyMonitor/        # Sicherheitsüberwachung
│   ├── LoadMonitor/          # Last-/Wind-Statistik
│   └── TempSensors/          # Temperaturmessung
├── platformio.ini            # PlatformIO-Konfiguration
└── .github/workflows/        # CI/CD (GitHub Actions)
```

## Lizenz

Dieses Projekt ist Open Source. Siehe Repository für Lizenzdetails.

## Autor

- **maggo1404** - [GitHub](https://github.com/maggo1404)

---

*Für Fragen oder Probleme bitte ein Issue im GitHub Repository erstellen.*