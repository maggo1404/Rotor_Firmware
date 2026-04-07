#include <Arduino.h>
#include <math.h>

#include <Preferences.h>

#include "HalBoard.h"
#include "MotorMcpwm.h"
#include "Rs485Proto.h"
#include "Rs485Dispatcher.h"

#include "EncoderAxis.h"
#include "TempSensors.h"
#include "LoadMonitor.h"

#include "HomingController.h"
#include "SafetyMonitor.h"
#include "MotionController.h"

// ============================================================================
// Hardware
// ============================================================================
#define PWM_GPIO_IN1 6
#define PWM_GPIO_IN2 7
#define ENABLE_GPIO 21
#define PWM_FREQUENCY_HZ 21000

#define ENC_PIN_A 10
#define ENC_PIN_B 11
#define ENC_PIN_Z 5
#define ENC_Z_ACTIVE_HIGH true

// ============================================================================
// RS485
// ============================================================================
// RS485 Slave-ID dieses Geraets.
// Muss zum Master passen. Beispiel: Master erwartet Slave 20 -> hier 20 setzen.
static uint8_t g_slaveId = 20;

// Preferences/NVS: persistente Konfiguration (wird nur bei SET-Kommandos geschrieben).
static Preferences g_prefs;

// Merker: konnten Preferences/NVS erfolgreich geoeffnet werden?
// (Wenn nicht, laufen wir mit Defaults, aber persistentes Speichern geht nicht.)
static bool g_prefsOk = false;

// RS485 UART Baudrate. Muss bei Master und Slave identisch sein.
// 115200 ist ein guter Standard; hoeher = weniger Latenz, aber stoeranfaelliger bei schlechter Leitung.
static const uint32_t RS485_BAUD = 115200;

// RS485 Sende-Timing (Treiber-Umschaltung).
// TPRE = Wartezeit vor Senden (DIR auf TX), TPOST = Wartezeit nach Senden (DIR zurueck).
// Halbduplex: nach TX muss der Wandler sicher im RX sein, bevor der Master antwortet —
// zu kurz -> Kollisionen / keine ACKs (Master sendete frueher ohne Inter-Frame-Gap).
static const uint16_t RS485_TPRE_US  = 550;
static const uint16_t RS485_TPOST_US = 550;

// Groesserer HardwareSerial-RX-Puffer fuer Serial1.
// Der Wert wird absichtlich VOR rs485.begin() gesetzt, damit die UART auch bei
// kurzen Burst-Phasen mehrere Telegramme puffern kann, waehrend der RX-Task sie
// in Ruhe in die Software-Queue uebernimmt.
static const size_t RS485_RX_BUFFER_BYTES = 2048;

// RS485 Frames mitloggen (Serial Debug). Zum Tuning/Fehlersuche sinnvoll, spaeter ggf. false setzen.
static bool g_logRs485Frames = false;

// ============================================================================
// Debug
// ============================================================================
// Debug-Ausgaben auf Serial (USB).
// true = viele Statuszeilen; kann Loop verlangsamen.
// Wenn Regelung komisch wirkt: testweise false setzen (Debug kann Timing beeinflussen).
static bool g_debug = false;

// ============================================================================
// Neustart (wird durch RS485 SET-Kommandos angefordert)
// ============================================================================
// Manche Parameter wirken erst sauber nach setup() (EncoderType / ExpectedCounts).
// Der Rs485Dispatcher setzt diese Flags, der eigentliche Neustart passiert in loop()
// (mit kurzer Verzoegerung, damit die ACK-Nachricht sicher rausgeht).
static bool g_restartRequested = false;
static uint32_t g_restartAtMs = 0;

// ============================================================================
// Einheiten-Konventionen (wichtig fuer alle Einstellwerte)
// ============================================================================
// - "Deg01" bedeutet Grad * 100  ->  1 Deg01 = 0,01deg
//   Beispiel: 2000 Deg01 = 20,00deg
// - PWM/Speed in Prozent (%):
//   * Bei den PWM-Werten sind 0..100 gemeint (Vorzeichen kommt durch Fahrtrichtung).
// - Geschwindigkeiten sind in deg/s (Grad pro Sekunde).
//
// Tipp zum Tuning:
// - Wenn Bewegungen "hart" wirken: Rampenlaenge (g_rampDistDeg) erhoehen und/oder g_posKpSpeedPerDeg senken.
// - Wenn es zu traege wird: Rampenlaenge verringern oder Kp erhoehen (vorsichtig, sonst Schwingen).

// ============================================================================
// Achsenbereich / Wrap
// ============================================================================
// Wrap/Modulo-Verhalten der Achse.
// FALSE (gewuenscht): Achse hat harte Grenzen (g_axisMinDeg01..g_axisMaxDeg01), keine 0/360deg-Ueberlaeufe.
// TRUE waere nur sinnvoll bei Endlos-Achse ohne Endschalter/Anschlaege.
static bool g_axisWrapEnabled = false;

// Gueltiger Positionsbereich (in Deg01) fuer Plausibilisierung/Endstop-Restrict.
// Beispiel: 0..36000 = 0..360deg.
// Bei mechanischen Limits entsprechend enger setzen.
static int32_t g_axisMinDeg01 = 0;

// Oberes Achsenlimit (in Deg01). Siehe g_axisMinDeg01.
static int32_t g_axisMaxDeg01 = 36000;

// Bereichs-Offset (Deg01) fuer mechanischen Versatz des rechten Endschalters.
//
// Problem:
// - Linker Endschalter definiert 0,00deg.
// - Rechter Endschalter kann mechanisch zu spaet ausloesen, z.B. erst bei 362,50deg.
//
// Ziel:
// - Reale Endschalterstrecke kann z.B. 362,50deg betragen.
// - Logischer Arbeitsbereich soll jedoch 0..360,00deg bleiben.
// - UND: "0" soll nicht in den linken Endschalter fahren.
//
// Umsetzung:
// - Wir legen den logischen Bereich symmetrisch in die reale Endschalterstrecke.
//   Beispiel off=2,50deg:
//     links 1,25deg Abstand, rechts 1,25deg Abstand.
// - Das passiert rein in der Umrechnung Counts<->Deg01 (EncoderAxis), Homing
//   selbst arbeitet weiterhin in Counts.
//
// Konvention:
// - Deg01: Grad * 100  ->  2,50deg = 250
//
// Wirkung:
// - Der Offset wird in EncoderAxis als Skalierungs-Korrektur umgesetzt
//   (Counts<->Deg01) und ist damit unabhaengig vom verwendeten Encoder (160k/50k).
//
// RS485:
// - SETDGOFFSET:<wert> (z.B. 2,50)  -> sofort wirksam + persistiert (Preferences)
// - GETDGOFFSET -> aktuellen Wert
static int32_t g_dgOffsetDeg01 = 250;

// Umkehrspiel/Backlash (in Deg01).
// Wird nach erfolgreichem Homing automatisch gemessen und hier gespeichert.
// Wird bei jedem Richtungswechsel als Offset auf das Ziel addiert, damit die Position nach dem Umkehren stimmt.
// Hinweis: Vor Homing ist der Wert 0, danach typischerweise einige Deg01 (z.B. 10..200).
static int32_t g_backlashDeg01 = 0;

// ============================================================================
// Homing
// ============================================================================
// Homing-PWM (in %):
// - Fast: zuegig Richtung Endschalter fahren, um ihn sicher zu finden.
// - Approach: langsamer, damit der Schaltpunkt reproduzierbar ist.
// - Backoff: vom Endschalter wegfahren (loesen).
// Beispiel: Fast 80..100%, Approach 20..40%, Backoff 20..40%.
static float g_homeFastPwmPercent     = 100.0f;
// SEEK MIN (linker Endschalter) - Fast-Segment:
// Wir wissen am Start nicht, wie weit der linke Endschalter entfernt ist.
// Daher fahren wir nach einer kurzen Anfahr-Rampe mit einem separaten,
// konstanten PWM-Wert weiter, bis der Endschalter erreicht ist.
// Typischer Wert: 50..70% (hier Default 60%).
static float g_homeSeekMinPwmPercent = 60.0f;
// g_homeBackoff (PWM in %)
// Dieser Wert wird fuer den Rueckzug vom Endschalter UND fuer die "Touch"-Fahrten
// (erneutes Einfahren in den Endschalter) verwendet.
//
// Idee:
// - g_minPwm ist die allgemeine Mindest-PWM (Creep/Anlauf), wird auch als Approach genutzt.
// - g_homeBackoff darf (wenn noetig) hoeher sein, damit der Endschalter bei hoher Haftreibung
//   sicher wieder ausgeloest und erneut sauber gedrueckt werden kann.
//
// Typische Werte: 20..40% (je nach Mechanik)
static float g_homeBackoff = 22.0f;

// Homing: nach Referenzfahrt auf 0deg zurueckfahren?
// true = nach Homing ist Position 0deg (Referenz = Nullpunkt).
// false = Position bleibt am Referenzpunkt (Endschalter-Position).
static bool g_homeReturnToZero = true;

// Homing-Timeout pro Segment (ms).
// Zu klein -> Homing bricht ab, zu gross -> Fehler wird spaet erkannt.
// Beispiel: bei langsamer Mechanik 60000 ms.
static uint32_t g_homeTimeoutMs = 120000;

// ------------------------------------------------------------
// Homing-Rampen in COUNTS (Encoder-unabhaengig ueber Skalierung)
// ------------------------------------------------------------
// Wir unterscheiden zwei typische Encoder:
// - Ring/Abtrieb: ca. 160000 Counts pro 360deg
// - Motorachse:   ca.  50000 Counts pro 360deg
// Diese Erwartungswerte sollen leicht anpassbar bleiben.
static int32_t g_homeExpectedCountsRing  = 160000;
static int32_t g_homeExpectedCountsMotor = 28600;

// Basiswerte fuer Ring-Encoder (160000 Counts):
// - SEEK MIN: Anfahr-Rampe nach links (minPWM -> g_homeSeekMinPwmPercent)
// - SEEK MAX: Beschleunigungsrampe nach rechts (minPWM -> g_homeFastPwmPercent)
// - SEEK MAX: ab dieser Count-Position wieder auf minPWM abbremsen
static int32_t g_homeSeekMinRampCountsRing     = 5000;
// SEEK MIN: zusaetzliche Abbrems-Rampe NACH Ausloesen von END_MIN.
// Viele Endschalter lassen sich ein Stueck ueberfahren, bevor ein harter Anschlag kommt.
// Dieser Wert ist das "sichere" Ueberfahr-Stueck fuer den Ring-Encoder (160000 Counts/360deg).
// Bei Motor-Encoder wird automatisch skaliert.
// Default: 2000 Counts (Ring) -> ca. 625 Counts bei 50000 Counts/360deg.
static int32_t g_homeSeekMinOverrunCountsRing = 2000;
static int32_t g_homeSeekMaxAccelRampCountsRing = 13300;
static int32_t g_homeSeekMaxDecelStartCountsRing = 150000;

static int32_t g_homeSeekMaxDecelRampCountsRing = 10000;
// Rueckweg zur 0 (Return-to-Zero) - Rampen in Grad.
// 30deg Anfahr-Rampe, 30deg vor Ziel abbremsen.
static float g_homeReturnRampDeg = 30.0f;

// ============================================================================
// Encoder
// ============================================================================
// Encoder-Typ (muss zur Hardware passen!):
// - ENCTYPE_MOTOR_AXIS:
//     Encoder sitzt auf der Motorachse.
//     -> Das mechanische Umkehrspiel (Getriebe/Spiel) liegt NACH dem Encoder und muss softwareseitig
//        beruecksichtigt werden (Backlash-Kompensation macht hier Sinn).
//     -> Z-Signal kann vorhanden sein, ist aber je nach Encoder/Anbau nicht zwingend.
// - ENCTYPE_RING_OUTPUT:
//     Ring-Encoder sitzt auf der Ausgangsachse.
//     -> Du misst direkt die Abtriebsposition (Spiel wird "mitgemessen"), daher in der Regel KEINE
//        zusaetzliche Umkehrspiel-Kompensation noetig.
//     -> In unserem Projekt nutzt dieser Modus zusaetzlich das Z-Signal (Index) zur Korrektur/CPR-Lernen.
// Hinweis: falscher Typ fuehrt zu falscher Z-Auswertung und ggf. falschem Umkehrspiel-Verhalten.
static EncoderType g_encType = ENCTYPE_MOTOR_AXIS;

// Encoder-Modus (Aufloesung / Entstoerung):
// - ULTRA_MODE_SINGLE:
//     zaehlt alle Pulse (maximale Aufloesung).
// - ULTRA_MODE_HALF:
//     zaehlt jeden zweiten Schritt -> halbe Aufloesung.
// - ULTRA_MODE_FULL:
//     Quadratur / "Vollschritt": zaehlt jeden 4. Puls -> geringste Aufloesung, dafuer am unempfindlichsten gegen Stoerungen.
// Wichtig: Muss zur Verdrahtung/Decoder passen. Falscher Modus -> falsche Schritte/Skalierung.
static UltraEncoderMode g_encMode = ULTRA_MODE_SINGLE;

// ============================================================================
// Bedienung / Defaults
// ============================================================================
// Handspeed (% PWM) fuer lokale Tasterbedienung (links/rechts) ohne Rampe.
// Diese Funktion ist bewusst "hart" (keine Beschleunigungsrampe), damit es sich direkt anfuehlt.
// Beispiel: 15% fuer sehr sanft, 30% fuer zuegig.
static float g_handSpeedPercent  = 25.0f;

// ============================================================================
// Timeouts
// ============================================================================
// Timeout fuer Positionsfahrt (ms).
// Wenn Ziel in dieser Zeit nicht erreicht wird -> Fehler/Abbruch (Safety).
// Beispiel: sehr langsame Fahrt: 120000 ms.
static uint32_t g_posTimeoutMs = 60000;

// Deadman / Keepalive fuer RS485-Betrieb (ms).
// 0 = deaktiviert.
// >0 = waehrend Homing oder Positionsfahrt gilt:
//      - Deadman darf NUR dann zuschlagen, wenn WAEHREND EINER BEWEGUNG
//        (PWM != 0 angefordert) laenger als X ms kein gueltiges Kommando
//        vom Master empfangen wurde.
//      - In Haltephasen (PWM ~ 0) soll es keinen Timeout geben.
// Beispiel: 2000 -> 2 Sekunden ohne Poll waehrend Bewegung => SE_TIMEOUT.
static uint32_t g_cmdTimeoutMs = 2000;

// ============================================================================
// Safety
// ============================================================================
// Endstop-/Achslimit-Restriktion.
// true = Bewegungen werden auf [axisMin..axisMax] begrenzt und Endstop-Zustaende werden sicher behandelt.
// false = nur fuer spezielle Tests (nicht empfohlen).
static bool g_restrictEndstops = true;

// Endschalter-Entprellung / Stoerunterdrueckung (ms)  (EEPROM-tauglich)
//
// Problem:
// - Beim Homing ist ein Endschalter oft gedrueckt und wir fahren anschliessend "Backoff" wieder heraus.
// - Durch EMV/PWM-Schalten koennen Endschalter-Eingaenge sehr kurze Glitches bekommen.
// - Ohne Entprellung kann das zu falschen Endstop-Fehlern (z.B. ERR11) fuehren.
//
// Loesung:
// - Ein Endschalter muss g_endstopDebounceMs lang stabil sein, bevor er als "aktiv" gilt.
// - 0 = Entprellung aus (nicht empfohlen).
//
// Richtwerte:
// - 20..50 ms ist meist ideal.
// - Wenn du extrem traege/alte Schalter hast: 60..100 ms.
static uint32_t g_endstopDebounceMs = 30;

// ============================================================================
// Stromueberwachung (IS) in mV (EEPROM-tauglich)
// ============================================================================
//
// WICHTIG: Es werden mV-Werte der Strommess-Eingaenge ausgewertet (nicht Ampere!).
// Je nach Board/Hardware entspricht ein bestimmter mV-Wert einem bestimmten Motorstrom.
// Fuer eine sinnvolle Einstellung musst du im Debug beobachten, welche IS-mV-Werte
// bei normaler Fahrt, bei Anlaufspitze und bei Blockade auftreten.
//
// Typisches Vorgehen:
// 1) Strommessung aktiv lassen (DEFAULT: an). Etappe2 hatte g_debug=true -> IS lief mit;
//    hier ist g_currentMonitorEnabled absichtlich NICHT an g_debug gekoppelt, sonst bleiben
//    GETIS/GETACCBINS dauerhaft 0 wenn g_debug=false.
// 2) Normale Fahrten + Richtungswechsel ausfuehren und IS-Werte beobachten (optional g_debug=true).
// 3) g_isHardStopMv so waehlen, dass normale Spitzen NICHT ausloesen, Blockade aber sicher ausloest.
// 4) g_isSoftWarnMv typischerweise bei ca. 70..90% von g_isHardStopMv setzen.
//
static bool     g_currentMonitorEnabled = true;
static uint32_t g_isSoftWarnMv = 250;     // Warnschwelle (mV) -> nur Warnung/Log, KEIN Stop
static uint32_t g_isHardStopMv = 350;    // Abschaltschwelle (mV) -> Fault + Motor aus
static uint32_t g_isGraceMs = 200;        // Schonzeit nach Start/Umkehr (ms): HardStop wird ignoriert
static uint32_t g_isHardHoldMs = 60;      // HardStop muss so lange ueberschritten sein (ms)
static uint8_t  g_isFilterLen = 16;       // Mittelwert-Laenge (1..32). Groesser=ruhiger, aber langsamer
static uint32_t g_isSampleIntervalMs = 5; // Abtastintervall (ms). Kleiner=schneller, kann mehr rauschen

// Viele Boards/Treiber liefern einen Grundoffset (z.B. ~120mV), auch wenn die H-Bruecke AUS ist.
// Damit SoftWarn/HardStop gut einstellbar sind, ziehen wir diesen Offset automatisch ab.
//
// Ablauf:
// - Beim Start (setup) wird bei MOTOR AUS / PWM=0 der IS1/IS2-Grundwert gemessen.
// - Dieser Wert wird waehrend des Betriebs immer abgezogen (Clamp auf 0mV).
//
// Optional kannst du zusaetzlich einen kleinen manuellen Trim setzen (z.B. wenn ein Kanal konstant
// 5..10mV daneben liegt). Standard: 0.
static bool     g_isAutoCalEnabled = true;   // true: Offset beim Start automatisch messen
static uint32_t g_isAutoCalSettleMs = 250;   // Wartezeit nach Start (ms), damit ADC/Signale stabil sind
static uint16_t g_isAutoCalSamples = 64;     // Anzahl Samples fuer die Offset-Messung
static uint32_t g_isAutoCalSampleIntervalMs = 5; // Abstand zwischen Samples (ms)

static int32_t  g_isTrim1Mv = 0;             // Manueller Zusatz-Offset fuer IS1 (mV), kann auch negativ sein
static int32_t  g_isTrim2Mv = 0;             // Manueller Zusatz-Offset fuer IS2 (mV)

// Laufzeit: gemessener Offset (wird im setup bestimmt)
static uint32_t g_isAutoOffset1Mv = 0;
static uint32_t g_isAutoOffset2Mv = 0;


// Richtungskonvention der PWM fuer Endschalter-Logik (EEPROM-tauglich)
//
// Safety muss wissen, welches PWM-Vorzeichen mechanisch nach links/rechts faehrt,
// damit ein gedrueckter Endschalter nur die "falsche" Richtung blockiert.
//
// true  (Standard):
//   duty > 0  -> Richtung RIGHT-Endschalter (MAX)
//   duty < 0  -> Richtung LEFT-Endschalter  (MIN)
//
// false (invertiert):
//   duty > 0  -> Richtung LEFT-Endschalter  (MIN)
//   duty < 0  -> Richtung RIGHT-Endschalter (MAX)
//
// Einstellhilfe:
// - Wenn beim Homing das "Backoff" (aus dem gedrueckten Endschalter herausfahren)
//   mit ERR11 (SE_ENDSTOP) blockiert wird, dann ist diese Zuordnung invertiert.
//   -> Wert umschalten.
//
// Hinweis:
// Die Stall-Werte (Timeout + MinCounts) gelten bewusst fuer ALLE Bewegungen (Homing + Positionsfahrt).
// Das vereinfacht das Tuning deutlich. Fuer Sonderfaelle gibt es:
// - Stall-Arming-Schwelle (g_minStallPwm) -> verhindert Fehlalarme bei Mini-Schritten
// - "Micro-Stall" intern -> erkennt echte Blockaden auch bei sehr kleinen Sollschritten

// Safety: Richtungskonvention fuer Endschalter-Logik
// true  : duty > 0 bewegt Richtung RIGHT-Endschalter (MAX)
// false : duty > 0 bewegt Richtung LEFT-Endschalter  (MIN)
// Tuning: Wenn beim Homing das Backoff aus dem gedrueckten Endschalter blockiert wird (ERR11), umschalten.
static bool g_safetyDutyPositiveMovesRightEndstop = true;

// ============================================================================
// Arrival / Zielerkennung
// ============================================================================
// Ziel-Toleranz (Deg01): Innerhalb dieser Abweichung gilt das Zielfenster als erreicht.
// Beispiel: 2 = 0,02deg (sehr genau), 50 = 0,50deg (grober, aber schneller).
//
// Besonderheit (Feinphase):
// - Sobald das Zielfenster erreicht wird, versucht der Controller automatisch genau EINMAL,
//   exakt auf err==0,00deg zu kommen (zweiter "Treffer").
// - Wenn das nicht klappt, bleibt er trotzdem sicher im Zielfenster und meldet nach Haltezeit "fertig".
static int32_t  g_arriveTolDeg01 = 2;          // 0,02deg

 // Haltezeit (ms), die innerhalb der Toleranz erreicht werden muss.
 // Erhoehen, wenn du bei Vibrationen/Federung ein Flattern um die Toleranz siehst.
 // Senken, wenn es schneller "fertig" melden soll.
static uint32_t g_arriveHoldMs = 200;

// Geschwindigkeits-Schwelle (deg/s) fuer "stillstehend" bei Zielerkennung.
// Wenn zu klein -> er wartet lange, wenn zu gross -> er meldet zu frueh fertig.
// Beispiel: 0.20 ist konservativ; 0.40 meldet schneller fertig.

// Speed Messung
// Intervall fuer Geschwindigkeitsmessung aus Encoder-Counts (ms).
// Kleiner = reagiert schneller, kann aber rauschen.
// Groesser = glatter, aber traeger.
// Beispiel: 20..40 ms ist meist gut.

// ============================================================================
// LED
// ============================================================================
// LED Blinken im Idle (ms pro Periode).
static uint32_t g_ledIdlePeriodMs = 1000;
// LED Blinken waehrend Bewegung (ms pro Periode).
static uint32_t g_ledMovePeriodMs = 200;

// ============================================================================
// Motion / Regler
// ============================================================================
// Grobfahrt (lange Strecken) laeuft im PWM-Profil-Modus:
// - Ramp-Up -> Cruise -> Ramp-Down
// - Profil ist positionsgefuehrt (Encoder), aber OHNE Speed-PI
//
// Stellschrauben sind absichtlich minimal:
// - g_pwmMaxAbs      : maximale PWM in der Grobfahrt
// - g_rampDistDeg    : Rampenlaenge (Grad) fuer Anfahren + Abbremsen
// - g_minPwm         : Mindest-PWM zum sicheren Anlaufen (Haftreibung) + Feinphase-Creep
// - g_minStallPwm    : Stall-Arming-Schwelle (muss etwas ueber g_minPwm liegen)
//
// Feinphase (Zielbereich) bleibt separat: Creep-PWM + aktive Bremse.

// Rampenstrecke in Grad (Beschleunigen *und* Abbremsen).
// - Grosse Werte -> weich, wenig Lastspitzen, mehr Weg bis zur vollen Leistung
// - Kleine Werte -> knackig, kann ruckiger werden
// Diese Rampe gilt auch fuer STOP/Richtungswechsel (virtuelles Bremsziel).
static float  g_rampDistDeg    = 30.0f;

// Feinzone (Deg01): ab dieser Reststrecke schalten wir von Grob auf Fein.
// Beispiel: 200 (2,00deg) .. 1000 (10,00deg)
static int32_t g_fineWindowDeg01 = 200;

// Minimal-PWM (% absolut) - gemeinsamer Basiswert fuer:
// 1) Feinphase: konstantes "Creep-PWM" (damit der Motor in der Feinzone nicht klebt)
// 2) Anlauf/Kick: Mindest-PWM, um Haftreibung sicher zu ueberwinden (kurze Wege starten sauber)
//
// Tuning:
// - Wenn er in der Feinphase stehen bleibt oder kurze Wege nicht anlaufen -> erhoehen.
// - Wenn er in der Feinphase zu aggressiv wirkt -> leicht senken (aber nicht so weit, dass er klebt!).
static float   g_minPwm = 25.0f;

// Stall-Ueberwachung aktiv (gilt fuer Homing + Positionsfahrt)
// true  = Stall aktiv (Blockade wird erkannt)
// false = aus (nur fuer Fehlersuche empfohlen)
static bool g_stallMonitorEnabled = true;

// Stall-Arming PWM (% absolut): ab dieser PWM erwartet die Stall-Erkennung innerhalb g_stallTimeoutMs
// eine "echte" Bewegung (g_stallMinCounts).
//
// Warum getrennt von g_minPwm?
// - Bei extrem kleinen Sollschritten (0,01deg..0,05deg) kann es sein, dass der Motor bewusst nur mit g_minPwm
//   arbeitet. Wuerde der Stall schon bei g_minPwm starten, gaebe es bei diesen Mini-Schritten unnoetig SE_STALL.
//
// Tuning:
// - g_minStallPwm sollte IMMER etwas groesser als g_minPwm sein (typisch +2 .. +5).
// - Wenn bei Mini-Schritten noch SE_STALL kommt -> g_minStallPwm erhoehen.
// - Wenn du Blockaden auch bei sehr langsamer/kleiner PWM schneller erkennen willst -> g_minStallPwm senken
//   (aber nicht unter g_minPwm!).
// Default bewusst etwas ueber g_minPwm, damit beim Creep (g_minPwm) keine
// falschen SE_STALL auftreten (z.B. beim rechten Endschalter-Suchlauf im Homing).
static float   g_minStallPwm = 25.0f;

// Stall Timeout (ms): innerhlab dieser Zeit muessen mindestens g_stallMinCounts Encoder-Counts kommen,
// wenn eine Bewegung erwartet wird (Duty >= g_minStallPwm).
//
// Tuning:
// - Hoeher: toleranter bei kleinen/zaehen Bewegungen, reagiert spaeter auf echte Blockaden.
// - Niedriger: schnelleres Fault bei Blockade, kann aber bei sehr zaehem Anlauf frueh ausloesen.
static uint32_t g_stallTimeoutMs = 2000;

// Stall MinCounts: Mindest-Encoder-Counts innerhalb g_stallTimeoutMs, die als "Bewegung" gelten.
//
// Tuning:
// - Hoeher: verlangt mehr Bewegung -> schnelleres Stall bei Blockade, aber kann bei sehr langsamer Bewegung frueh triggern.
// - Niedriger: toleranter, aber erkennt Blockade spaeter.
static uint32_t g_stallMinCounts = 10;


// Feinphase: wie lange die aktive Bremse gehalten wird (ms), wenn wir Ziel erreicht/ueberfahren haben
static uint32_t g_fineBrakeHoldMs = 80;

// Feinphase: optional "vor dem Ziel" bremsen (0,01deg), gueltig fuer + und - Richtung.
// 0 => nur noch bei err<=0 bremsen (spaet, viel Schwung bei kurzen Minus-Schritten).
// ~100..150 (1,0..1,5deg): Vorab-Bremse, weniger Ueberschiessen bei 1..2deg-Schritten.
static int32_t g_fineBrakeLeadDeg01 = 120;


// PWM Max (%): Harte PWM-Grenze fuer Positionsfahrten (SETPOSDG).
// Maximale PWM in der Grobfahrt:
// - g_pwmMaxAbsNv  : persistenter Default aus Preferences (gilt nach Neustart)
// - g_pwmMaxAbsCmd : Laufzeit-Sollwert (SETPWM, wird NICHT gespeichert)
// - g_pwmMaxAbs    : geglaetteter Istwert, den MotionController wirklich nutzt (rampt weich auf Cmd)
static float g_pwmMaxAbs = 100.0f;
static float g_pwmMaxAbsCmd = 100.0f;
static float g_pwmMaxAbsNv = 100.0f;


// ============================================================================
// Temperatur / LoadMonitor (Kalibrierung + Statistik)
// ============================================================================
// DS18B20 OneWire-Pin (Umgebung + optional Motor)
// Hinweis: Sensor wurde von Joerg an GPIO 2 getestet.
static uint8_t  g_tempOneWirePin = 2;     // GPIO2
static uint32_t g_tempIntervalMs = 1000;  // Messintervall (ms)

// Motor-Temperatursensor (DS18B20 #1) ist optional.
// - Default: AUS (Sensor nicht vorhanden)
// - Wenn deaktiviert: GETTEMPM liefert 0.
static bool g_motorTempSensorEnabled = true;

// Temperatur-Sensoren (DS18B20) koennen je nach Bus-Reihenfolge vertauscht sein.
// - false: Umgebung=Device0, Motor=Device1
// - true : Umgebung=Device1, Motor=Device0
// Wird in Preferences gespeichert (Key: "tsw") und kann per RS485 gesetzt werden.
static bool g_tempSwapSensors = true;

// Warnschwellen (GradC). <=0 deaktiviert die jeweilige Warnung.
static float g_tempWarnAmbientC = 0.0f;
static float g_tempWarnMotorC   = 0.0f;

// Kaelte-Kompensation: unterhalb dieser Temperatur darf die Reibung hoeher sein.
// Beispiel: <=5C sind +10% mehr Reibung OK.
static float g_coldTempDegC       = 5.0f;
static float g_coldExtraDragPct   = 10.0f;

// Zusaetzliche Ignorier-Rampe (SETCALIGNDG/cig); effektiv max(ramp, cal) fuer Live-Bins/Kalibrierung.
static float g_calIgnoreRampDeg = 10.0f;

// Mindestweg (Grad) fuer Live/Acc-Bins und GETLOADSTAT/GETWIND (SETSTATMINDG/smm).
static float g_statMinMoveDeg = 15.0f;

// GETACCBINS: separater Ignore (SETRAPDG/rap), Standard 5deg.
static float g_accIgnoreRampDeg = 5.0f;

// Mechanische Reibung / Getriebe: Schwellwerte
static float   g_dragWarnPct      = 25.0f; // Mittelwert-Schwellwert (%)
static float   g_dragWarnBinsPct  = 30.0f; // wie viele Bins muessen drueber sein (%)
static uint8_t g_dragPersistMoves = 3;     // wie viele grosse Fahrten hintereinander

// Wind-Erkennung
static float g_windPeakPct      = 60.0f; // Peak-Schwellwert (%)
static float g_windCoherenceMin = 55.0f; // Kohaerenz-Minimum (%)

// Wind- & Richtungssensor (RS485) aktivieren/deaktivieren
// 1 = aktiv (Sensor wird abgefragt)
// 0 = aus   (keine Abfragen, Rueckgabe immer 0)
static bool g_windEnable = true;

// ============================================================================
// Anemometer (Windmesser, analog 0..2V)
// ============================================================================
// Joerg: Windmesser an IO9 liefert 0..2V.
// - 0V  = 0 km/h (Windstille)
// - 2V  = 200 km/h
//
// GETANEMO liefert den aktuellen km/h-Wert (inkl. Offset) mit 1 Nachkommastelle.
// SETANEMO setzt einen +/- Offset (km/h), ebenfalls mit 1 Nachkommastelle.
//
// Der Offset wird in Preferences gespeichert, damit er nach Neustart erhalten bleibt.
static float g_anemoOffsetKmh = 0.0f;

// Windrichtung-Offset (Grad) fuer den RS485 Wind-/Richtungssensor.
// Wird bei Montage einmalig per SETWINDDIROF gesetzt und im NVS gespeichert.
static float g_windDirOffsetDeg = 0.0f;

// Antennen-Versatz 1..3 (Grad).
// Diese drei Werte werden nur im Rotor persistent gespeichert und anderen Controllern
// per RS485 bereitgestellt. Der Rotor selbst nutzt sie nicht aktiv fuer die Regelung.
static float g_antOffset1Deg = 0.0f;
static float g_antOffset2Deg = 0.0f;
static float g_antOffset3Deg = 0.0f;

// Frei gespeicherte Winkel 1..3 (Grad).
// Diese drei Werte werden nur im Rotor persistent gespeichert und per RS485
// fuer den Master bereitgestellt. Der Rotor selbst nutzt sie nicht aktiv.
static float g_angle1Deg = 0.0f;
static float g_angle2Deg = 0.0f;
static float g_angle3Deg = 0.0f;

// ============================================================================
// Homing Kick Retry (optional)
// ============================================================================
// Optionaler Mechanismus, falls der Rotor beim Homing manchmal "klebt" (Haftreibung).
// Idee: Nach einem Homing-Start duerfen einige kurze Kick-Versuche passieren,
// um Bewegung zu erzwingen, falls der Encoder keine Counts liefert.
// Standard: deaktiviert ueber g_homingKickMaxTries=0 oder indem der Dispatcher es nicht triggert.
//
// g_homingKickTries/NextMs sind Laufzeit-Variablen (nicht im EEPROM speichern).
// Einstellwert ist v.a. g_homingKickSpacingMs und g_homingKickMaxTries.
static uint8_t  g_homingKickTries = 0;          // Laufzeit: bisherige Versuche
static uint32_t g_homingKickNextMs = 0;         // Laufzeit: naechster erlaubter Kick-Zeitpunkt
static uint32_t g_homingKickSpacingMs = 150;    // Einstellwert: Mindestabstand zwischen Kicks (ms)
static uint8_t  g_homingKickMaxTries = 3;       // Einstellwert: max. Anzahl Kick-Versuche (0 = aus)

// ============================================================================
// Module
// ============================================================================
HalBoard board;
MotorMcpwm motor;

Rs485Proto rs485;
Rs485Dispatcher rs485Dispatcher;

SafetyMonitor safety;
SafetyConfig safetyCfg;

EncoderAxis encoder;
TempSensors temps;
LoadMonitor loadMon;
HomingController homing;
MotionController motion;

// ============================================================================
// Error Broadcast / LED State
// ============================================================================
static bool g_faultPrev = false;
static uint32_t g_lastErrBroadcastMs = 0;

static bool g_ledState = false;
static uint32_t g_ledLastToggleMs = 0;

// ============================================================================
// Helper
// ============================================================================
static bool isMovingByCommand(bool homingActive, float appliedDuty) {
  // Homing setzt PWM direkt am Motor. Fuer die LED ist entscheidend, ob tatsaechlich PWM anliegt.
  // Wenn appliedDuty ~ 0 ist, soll die LED im Idle-Pattern blinken.
  (void)homingActive;
  return (fabsf(appliedDuty) > 0.01f);
}

static void updateLbLed(uint32_t nowMs, bool moving, bool fault) {
  if (fault) {
    board.setLed(true);
    g_ledState = true;
    return;
  }

  uint32_t period = moving ? g_ledMovePeriodMs : g_ledIdlePeriodMs;
  if ((nowMs - g_ledLastToggleMs) >= (period / 2)) {
    g_ledLastToggleMs = nowMs;
    g_ledState = !g_ledState;
    board.setLed(g_ledState);
  }
}

// ============================================================================
// IS Auto-Offset Messung (ADC-Grundoffset)
// ============================================================================
// Misst den Grundoffset der IS-Eingaenge in mV (bei MOTOR AUS / PWM=0).
// Wir nutzen einen einfachen "trimmed mean": nach Sortierung werden 10% der
// kleinsten/groessten Samples verworfen. Das ist robust gegen Ausreisser.
static int cmpU16(const void* a, const void* b) {
  const uint16_t aa = *(const uint16_t*)a;
  const uint16_t bb = *(const uint16_t*)b;
  return (aa < bb) ? -1 : (aa > bb) ? 1 : 0;
}

static uint32_t measureIsOffsetMv(bool is1) {
  // ADC "warmup": erste Reads koennen (je nach Core/ADC) unplausibel sein.
  // Wir machen ein paar Dummy-Reads auf BEIDEN Kanaelen und verwerfen sie.
  for (uint8_t k = 0; k < 8; k++) {
    (void)board.readIs1mV();
    (void)board.readIs2mV();
    delay(2);
  }

  // Begrenzen, damit wir nicht zu viel RAM/Stack verbraten.
  uint16_t n = g_isAutoCalSamples;
  if (n < 8) n = 8;
  if (n > 128) n = 128;

  static uint16_t buf[128];

  // Wir sammeln "n" gueltige Samples. 0mV zaehlen wir nicht, weil das
  // typischerweise ein ADC-Glitch beim Start ist.
  uint16_t got = 0;
  uint16_t tries = 0;
  const uint16_t maxTries = (uint16_t)(n * 4);
  while (got < n && tries < maxTries) {
    tries++;
    uint32_t mv = is1 ? board.readIs1mV() : board.readIs2mV();
    if (mv == 0) {
      if (g_isAutoCalSampleIntervalMs > 0) delay(g_isAutoCalSampleIntervalMs);
      continue;
    }
    if (mv > 3300) mv = 3300;
    buf[got] = (uint16_t)mv;
    got++;

    if (g_isAutoCalSampleIntervalMs > 0) {
      delay(g_isAutoCalSampleIntervalMs);
    }
  }

  if (got < 8) {
    // Fallback: wenn wir wirklich kaum gueltige Samples bekommen, nutzen wir
    // den letzten Direkt-Read (kann auch 0 sein, aber dann ist ohnehin etwas faul).
    uint32_t mv = is1 ? board.readIs1mV() : board.readIs2mV();
    if (mv > 3300) mv = 3300;
    return mv;
  }

  n = got;

  qsort(buf, n, sizeof(uint16_t), cmpU16);

  uint16_t trim = n / 10; // 10%
  if ((uint16_t)(trim * 2) >= n) trim = 0;

  uint32_t sum = 0;
  uint16_t cnt = 0;
  for (uint16_t i = trim; i < (uint16_t)(n - trim); i++) {
    sum += buf[i];
    cnt++;
  }

  if (cnt == 0) {
    return (uint32_t)buf[n / 2];
  }
  // Rundung
  return (sum + (uint32_t)(cnt / 2)) / (uint32_t)cnt;
}

// ============================================================================
// Setup
// ============================================================================

// ============================================================================
// Preferences: Laden (Fallback = aktuelle Default-Werte)
// ============================================================================
// WICHTIG:
// - Wir lesen beim Start alle gespeicherten Werte aus Preferences (NVS).
// - Wenn ein Key noch nicht existiert, bleibt der aktuelle Default-Wert erhalten.
// - Schreiben passiert nur bei RS485-SET-Kommandos (nicht zyklisch), um Flash zu schonen.
static void loadPreferencesIntoGlobals() {
  // WICHTIG:
  // Preferences muessen in setup() bereits per begin("rotor", false) geoeffnet worden sein.
  // (Wir halten die Session offen, weil auch Rs485Dispatcher/LoadMonitor spaeter darauf schreiben.)
  if (!g_prefsOk) {
    // Wenn Preferences nicht initialisiert werden kann, bleiben Defaults aktiv.
    return;
  }

  g_slaveId          = g_prefs.getUChar("id",    g_slaveId);

  g_axisMinDeg01     = g_prefs.getInt("amin",    g_axisMinDeg01);
  g_axisMaxDeg01     = g_prefs.getInt("amax",    g_axisMaxDeg01);

  // Bereichs-Offset fuer rechte-Endschalter-Versatz
  g_dgOffsetDeg01    = g_prefs.getInt("dgo",     g_dgOffsetDeg01);

  g_homeFastPwmPercent = g_prefs.getFloat("hfp", g_homeFastPwmPercent);
  g_homeBackoff        = g_prefs.getFloat("hbo", g_homeBackoff);

  g_homeReturnToZero   = g_prefs.getBool("hrz",  g_homeReturnToZero);
  g_homeTimeoutMs      = g_prefs.getUInt("hto",  g_homeTimeoutMs);

  g_posTimeoutMs       = g_prefs.getUInt("pto",  g_posTimeoutMs);

  g_handSpeedPercent   = g_prefs.getFloat("hsp", g_handSpeedPercent);

  g_cmdTimeoutMs       = g_prefs.getUInt("dman", g_cmdTimeoutMs);

  g_isSoftWarnMv       = g_prefs.getUInt("iw",   g_isSoftWarnMv);
  g_isHardStopMv       = g_prefs.getUInt("imax", g_isHardStopMv);

  g_arriveTolDeg01     = g_prefs.getInt("atol",  g_arriveTolDeg01);

  g_rampDistDeg        = g_prefs.getFloat("ramp", g_rampDistDeg);

  g_minPwm             = g_prefs.getFloat("minp", g_minPwm);

  g_stallTimeoutMs     = g_prefs.getUInt("stto",  g_stallTimeoutMs);

  // --------------------------------------------------------------------------
  // Zusaetzliche Safety/Stall Parameter (neu: EEPROM + RS485)
  // --------------------------------------------------------------------------
  g_stallMonitorEnabled = g_prefs.getBool("sten", g_stallMonitorEnabled);
  g_minStallPwm         = g_prefs.getFloat("msp", g_minStallPwm);
  g_stallMinCounts      = g_prefs.getUInt("smc",  g_stallMinCounts);

  g_isGraceMs           = g_prefs.getUInt("igm",  g_isGraceMs);
  g_isHardHoldMs        = g_prefs.getUInt("ihm",  g_isHardHoldMs);
  g_isFilterLen         = g_prefs.getUChar("ifl", g_isFilterLen);

  // Defensive Clamps (damit alt/kaputt gespeicherte Werte nicht alles zerlegen)
  if (g_isFilterLen < 1) g_isFilterLen = 1;
  if (g_isFilterLen > 32) g_isFilterLen = 32;
  if (g_minStallPwm < 0.0f) g_minStallPwm = 0.0f;
  if (g_minStallPwm > 100.0f) g_minStallPwm = 100.0f;

  // Joerg: Stall-Arming muss ueber g_minPwm liegen, sonst kann bei Creep-PWM
  // (z.B. Endschalter-Suche im Homing) ein falscher Stall-Fehler entstehen.
  // Empfehlung: +2..+5% ueber g_minPwm.
  const float minStallSuggested = g_minPwm + 2.0f;
  if (g_minStallPwm < minStallSuggested) g_minStallPwm = minStallSuggested;
  if (g_minStallPwm > 100.0f) g_minStallPwm = 100.0f;

  // --------------------------------------------------------------------------
  // Zusaetzliche Homing/Encoder Parameter (neu: EEPROM + RS485)
  // --------------------------------------------------------------------------
  g_homeSeekMinPwmPercent = g_prefs.getFloat("hsm", g_homeSeekMinPwmPercent);
  if (g_homeSeekMinPwmPercent < 0.0f) g_homeSeekMinPwmPercent = 0.0f;
  if (g_homeSeekMinPwmPercent > 100.0f) g_homeSeekMinPwmPercent = 100.0f;

  g_homeExpectedCountsRing  = g_prefs.getInt("ecr", g_homeExpectedCountsRing);
  g_homeExpectedCountsMotor = g_prefs.getInt("ecm", g_homeExpectedCountsMotor);
  if (g_homeExpectedCountsRing <= 0)  g_homeExpectedCountsRing = 158000;
  if (g_homeExpectedCountsMotor <= 0) g_homeExpectedCountsMotor = 28000;

  // EncoderType: 1=MOTOR_AXIS, 2=RING_OUTPUT (Default: 1)
  // Kompatibilitaet: alter/ungesetzter Wert 0 wird als RING interpretiert.
  {
    uint8_t ect = g_prefs.getUChar("ect", 1);
    if (ect == 1) {
      g_encType = ENCTYPE_MOTOR_AXIS;
    } else {
      g_encType = ENCTYPE_RING_OUTPUT;
    }
  }



  // --------------------------------------------------------------------------
  // Temperatur / LoadMonitor (Kalibrierung + Statistik)
  // --------------------------------------------------------------------------
  // Temperatur-Warnschwellen
  g_tempWarnAmbientC   = g_prefs.getFloat("twa", g_tempWarnAmbientC);
  g_tempWarnMotorC     = g_prefs.getFloat("twm", g_tempWarnMotorC);

  // Temperatur-Sensoren tauschen (persistenter Swap)
  g_tempSwapSensors    = g_prefs.getBool("tsw", g_tempSwapSensors);

  // Kaelte-Kompensation
  g_coldTempDegC       = g_prefs.getFloat("cth", g_coldTempDegC);
  g_coldExtraDragPct   = g_prefs.getFloat("cpx", g_coldExtraDragPct);

  // Statistik/Kalibrierung
  g_calIgnoreRampDeg   = g_prefs.getFloat("cig", g_calIgnoreRampDeg);
  g_statMinMoveDeg     = g_prefs.getFloat("smm", g_statMinMoveDeg);
  g_accIgnoreRampDeg   = g_prefs.getFloat("rap", g_accIgnoreRampDeg);

  // Reibung/Wind-Schwellwerte
  g_dragWarnPct        = g_prefs.getFloat("drw", g_dragWarnPct);
  g_dragWarnBinsPct    = g_prefs.getFloat("drb", g_dragWarnBinsPct);
  g_dragPersistMoves   = g_prefs.getUChar("drn", g_dragPersistMoves);

  g_windPeakPct        = g_prefs.getFloat("wpk", g_windPeakPct);
  g_windCoherenceMin   = g_prefs.getFloat("wco", g_windCoherenceMin);

  // Anemometer-Offset (km/h)
  g_anemoOffsetKmh     = g_prefs.getFloat("ano", g_anemoOffsetKmh);

  // Windrichtung-Offset (Grad)
  g_windDirOffsetDeg   = g_prefs.getFloat("wdo", g_windDirOffsetDeg);

  // Antennen-Versatz 1..3 (Grad)
  g_antOffset1Deg      = g_prefs.getFloat("ao1", g_antOffset1Deg);
  g_antOffset2Deg      = g_prefs.getFloat("ao2", g_antOffset2Deg);
  g_antOffset3Deg      = g_prefs.getFloat("ao3", g_antOffset3Deg);

  // Frei gespeicherte Winkel 1..3 (Grad)
  g_angle1Deg          = g_prefs.getFloat("ag1", g_angle1Deg);
  g_angle2Deg          = g_prefs.getFloat("ag2", g_angle2Deg);
  g_angle3Deg          = g_prefs.getFloat("ag3", g_angle3Deg);

  // Wind- & Richtungssensor Enable (bool)
  g_windEnable         = g_prefs.getBool("wen", g_windEnable);
  // PWM-Max persistent laden
  g_pwmMaxAbsNv        = g_prefs.getFloat("maxp", g_pwmMaxAbsNv);
  // Laufzeit-Sollwert und geglaetteter Istwert initial = persistenter Wert
  g_pwmMaxAbsCmd       = g_pwmMaxAbsNv;
  g_pwmMaxAbs          = g_pwmMaxAbsNv;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // ------------------------------------------------------------------------
  // Board frueh initialisieren, damit wir die Service-Taster fuer einen
  // "Werksreset" (NVS auf Defaults) bereits beim Boot abfragen koennen.
  // ------------------------------------------------------------------------
  board.begin();
  delay(25); // Pullups kurz stabilisieren lassen

  // ------------------------------------------------------------------------
  // Preferences/NVS oeffnen (bleibt offen fuer Rs485Dispatcher/LoadMonitor)
  // ------------------------------------------------------------------------
  g_prefsOk = g_prefs.begin("rotor", false);

  // ------------------------------------------------------------------------
  // Werksreset per Hardware: Beide Handspeed-Taster beim Boot gedrueckt halten
  // -> NVS komplett loeschen (alle Keys im Namespace "rotor")
  // -> IMMER Neustart
  // ------------------------------------------------------------------------
  if (board.readServiceLeft() && board.readServiceRight()) {
    board.setLed(true);
    if (g_prefsOk) {
      // Alles loeschen: Konfiguration + Baseline/Statistik
      g_prefs.clear();
    }
    // Kleiner Delay, damit Flash/Serial sicher fertig sind
    delay(250);
    ESP.restart();
    return;
  }

  // Preferences laden (persistente Konfiguration)
  loadPreferencesIntoGlobals();

  // Persistente Wind-Offsets in den Wind-Sensor uebernehmen
  // (Speed-Offset bleibt kompatibel zum alten Analogsensor: km/h)
  board.setWindSpeedOffsetKmh(g_anemoOffsetKmh);
  board.setWindDirOffsetDeg(g_windDirOffsetDeg);

  // Wind-Sensor Enable aus Preferences anwenden
  board.setWindEnable(g_windEnable);

  // Den Wind-Task erst jetzt starten:
  // - Preferences sind geladen
  // - Offsets und Enable sind gesetzt
  // - damit spricht waehrend setup() nur ein einziger Kontext mit Serial2
  board.startWindTask();

  motor.begin(PWM_GPIO_IN1, PWM_GPIO_IN2, ENABLE_GPIO, PWM_FREQUENCY_HZ, 10UL * 1000UL * 1000UL);

  // Fuer die IS-Auto-Kalibrierung messen wir bei PWM=0.
  // Wichtig: Wir lassen den Treiber ENABLED, weil manche Hardware den IS-Pegel
  // im "enabled" Zustand geringfuegig anders haben kann als komplett disabled.
  // Bewegung ist ausgeschlossen, weil PWM=0 und keine Bremse aktiv ist.
  motor.enable(true);
  motor.brake(false);
  motor.stopPwm();

  // -------------------------
  // IS Auto-Kalibrierung (Grundoffset)
  // -------------------------
  // Manche Treiber/Boards zeigen einen stabilen IS-Offset, auch wenn die H-Bruecke aus ist.
  // Diesen Offset messen wir einmalig nach dem Start und ziehen ihn waehrend des Betriebs ab.
  if (g_isAutoCalEnabled) {
    delay(g_isAutoCalSettleMs);
    g_isAutoOffset1Mv = measureIsOffsetMv(true);
    g_isAutoOffset2Mv = measureIsOffsetMv(false);
  } else {
    g_isAutoOffset1Mv = 0;
    g_isAutoOffset2Mv = 0;
  }

  // Motor jetzt aktivieren (ab hier gilt: Safety entscheidet, was wirklich ausgegeben wird)
  motor.enable(true);
  motor.stopPwm();

  // -------------------------
  // Safety
  // -------------------------
  safetyCfg = SafetyConfig{};
  safetyCfg.restrictEndstops = g_restrictEndstops;
  safetyCfg.latchErrorOnEndstop = true;
  safetyCfg.cmdTimeoutMs = g_cmdTimeoutMs;

  // Endschalter-Entprellung
  safetyCfg.endstopDebounceMs = g_endstopDebounceMs;

  // Richtungskonvention PWM <-> Endschalter
  safetyCfg.dutyPositiveMovesRightEndstop = g_safetyDutyPositiveMovesRightEndstop;

  // Stromueberwachung (IS)
  safetyCfg.currentMonitorEnabled = g_currentMonitorEnabled;
  safetyCfg.isSoftWarnMv = g_isSoftWarnMv;
  safetyCfg.isHardStopMv = g_isHardStopMv;
  safetyCfg.isGraceMs = g_isGraceMs;
  safetyCfg.isHardHoldMs = g_isHardHoldMs;
  safetyCfg.isFilterLen = g_isFilterLen;
  safetyCfg.isSampleIntervalMs = g_isSampleIntervalMs;

  // IS-Offsets: gemessener Auto-Offset + optionaler manueller Trim
  int32_t off1 = (int32_t)g_isAutoOffset1Mv + g_isTrim1Mv;
  int32_t off2 = (int32_t)g_isAutoOffset2Mv + g_isTrim2Mv;
  if (off1 < 0) off1 = 0;
  if (off2 < 0) off2 = 0;
  safetyCfg.isOffset1Mv = (uint32_t)off1;
  safetyCfg.isOffset2Mv = (uint32_t)off2;
  // Blockade-/Stall-Erkennung
  safetyCfg.stallMonitorEnabled = g_stallMonitorEnabled;
  // Stall-Arming-Schwelle: getrennt von g_minPwm, damit Mini-Schritte nicht sofort SE_STALL ausloesen.
  // Empfehlung: g_minStallPwm = g_minPwm + 2..5
  safetyCfg.stallArmDutyAbs = g_minStallPwm;
  safetyCfg.stallTimeoutMs = g_stallTimeoutMs;
  safetyCfg.stallMinCounts = g_stallMinCounts;

  safety.begin(&board, safetyCfg);
  safety.setSerialLogging(g_debug);

  // -------------------------
  // Encoder
  // -------------------------
  EncoderAxisConfig ecfg;
  ecfg = EncoderAxisConfig{};

  ecfg.pinA = ENC_PIN_A;
  ecfg.pinB = ENC_PIN_B;

  ecfg.mode = g_encMode;
  ecfg.cpuCore = 0;
  ecfg.serviceIntervalUs = 200;
  ecfg.glitchNs = 200;

  ecfg.encType = g_encType;

  // Bereichs-Offset fuer rechte-Endschalter-Versatz (siehe g_dgOffsetDeg01)
  ecfg.rangeDegOffsetDeg01 = g_dgOffsetDeg01;

  if (g_encType == ENCTYPE_RING_OUTPUT) {
    ecfg.zEnabled = true;
    ecfg.zPin = ENC_PIN_Z;
    ecfg.zActiveHigh = ENC_Z_ACTIVE_HIGH;

    ecfg.zMinIntervalUs = 2000;
    ecfg.zMinAbsStepsBetween = 200;

    ecfg.zCorrEnabled = true;
    ecfg.zExpectedStepsBetweenZ = 2000;
    ecfg.zMaxAbsErrorSteps = 20;
    ecfg.zCorrGain = 1.0f;
  } else {
    ecfg.zEnabled = false;
    ecfg.zCorrEnabled = false;
  }

  ecfg.countsPerRevActual = 0;
  if (!encoder.begin(ecfg)) {
    if (g_debug) Serial.println("FEHLER: Encoder begin() fehlgeschlagen!");
  }

  // -------------------------
  // Homing
  // -------------------------
  HomingConfig hcfg;
  hcfg = HomingConfig{};
  hcfg.fastPwmPercent = g_homeFastPwmPercent;
  hcfg.seekMinPwmPercent = g_homeSeekMinPwmPercent;

  // Erwartete Counts pro 360deg (fuer Rampen-Skalierung):
  // - Ring/Abtrieb: g_homeExpectedCountsRing
  // - Motorachse:   g_homeExpectedCountsMotor
  int32_t expectedCounts = (g_encType == ENCTYPE_MOTOR_AXIS) ? g_homeExpectedCountsMotor : g_homeExpectedCountsRing;
  if (expectedCounts <= 0) expectedCounts = g_homeExpectedCountsRing;
  hcfg.expectedCountsPerRevHint = expectedCounts;

  // Rampen-Counts aus Ring-Basiswerten ableiten.
  // Skalierung linear ueber den Erwartungswert.
  const float scale = (float)expectedCounts / (float)g_homeExpectedCountsRing;
  hcfg.seekMinRampCounts = (int32_t)lroundf((float)g_homeSeekMinRampCountsRing * scale);
  hcfg.seekMinOverrunCounts = (int32_t)lroundf((float)g_homeSeekMinOverrunCountsRing * scale);
  hcfg.seekMaxAccelRampCounts = (int32_t)lroundf((float)g_homeSeekMaxAccelRampCountsRing * scale);
  hcfg.seekMaxDecelStartCounts = (int32_t)lroundf((float)g_homeSeekMaxDecelStartCountsRing * scale);
  hcfg.seekMaxDecelRampCounts = (int32_t)lroundf((float)g_homeSeekMaxDecelRampCountsRing * scale);
  if (hcfg.seekMinRampCounts < 1) hcfg.seekMinRampCounts = 1;
  if (hcfg.seekMinOverrunCounts < 0) hcfg.seekMinOverrunCounts = 0;
  if (hcfg.seekMaxAccelRampCounts < 1) hcfg.seekMaxAccelRampCounts = 1;
  if (hcfg.seekMaxDecelStartCounts < 0) hcfg.seekMaxDecelStartCounts = 0;
  if (hcfg.seekMaxDecelRampCounts < 1) hcfg.seekMaxDecelRampCounts = 1;
  if (hcfg.seekMaxDecelStartCounts > expectedCounts) hcfg.seekMaxDecelStartCounts = expectedCounts;

  hcfg.returnRampDeg = g_homeReturnRampDeg;
  // Approach (langsam) nutzt bewusst g_minPwm (Creep-Minimum), damit der Motor sicher laeuft
  // und der Schaltpunkt des Endschalters reproduzierbar ist.
  // Backoff bleibt separat ueber g_homeBackoff einstellbar.
  hcfg.approachPwmPercent = g_minPwm;
  hcfg.backoffPwmPercent = g_homeBackoff;
  hcfg.returnToZero = g_homeReturnToZero;
  hcfg.segmentTimeoutMs = g_homeTimeoutMs;
  homing.begin(&board, &motor, &encoder, hcfg);

  // -------------------------
  // Motion
  // -------------------------
  MotionConfigPointers mcfg;
  mcfg = MotionConfigPointers{};

  mcfg.handSpeedPercent = &g_handSpeedPercent;

  mcfg.posTimeoutMs = &g_posTimeoutMs;

  mcfg.arriveTolDeg01 = &g_arriveTolDeg01;
  mcfg.arriveHoldMs = &g_arriveHoldMs;

  mcfg.speedMeasIntervalMs = nullptr; // optional, Default in MotionController
  // Debug-Schalter fuer MotionController: vMeas und aehnliches nur berechnen, wenn Debug aktiv ist.
  mcfg.debugEnabled = &g_debug;
  mcfg.rampDistDeg = &g_rampDistDeg;

  mcfg.fineWindowDeg01 = &g_fineWindowDeg01;
  // Feinphase-Creep-PWM: wir nutzen g_minPwm als gemeinsamen Wert
  mcfg.finePwmAbs = &g_minPwm;
  mcfg.fineBrakeHoldMs = &g_fineBrakeHoldMs;
  mcfg.fineBrakeLeadDeg01 = &g_fineBrakeLeadDeg01;

  mcfg.pwmMaxAbs = &g_pwmMaxAbs;
  // Mindest-PWM zum Anlaufen: identisch mit g_minPwm
  mcfg.pwmKickMinAbs = &g_minPwm;
  mcfg.pwmSlewPerSec = nullptr; // optional, Default in MotionController
  mcfg.pwmSlewAutoFromRamp = nullptr; // Auto-Slew hier ohne Bedeutung
  mcfg.backlashDeg01 = &g_backlashDeg01;

  mcfg.axisWrapEnabled = &g_axisWrapEnabled;
  mcfg.axisMinDeg01 = &g_axisMinDeg01;
  mcfg.axisMaxDeg01 = &g_axisMaxDeg01;

  motion.begin(&board, &motor, &encoder, &homing, mcfg);


  // -------------------------
  // Temperatur-Sensoren (DS18B20)
  // -------------------------
  // Sensor 0: Umgebung (GETTEMPA)
  // Sensor 1: Motor    (GETTEMPM) -> optional (g_motorTempSensorEnabled)
  (void)temps.begin(g_tempOneWirePin, g_motorTempSensorEnabled, g_tempIntervalMs);

  // Falls die OneWire-Reihenfolge der Sensoren nicht passt: logische Zuordnung tauschen.
  temps.setSwapTemp(g_tempSwapSensors);

  // -------------------------
  // LoadMonitor (Kalibrierung + Statistik)
  // -------------------------
  // - Baseline (SETCAL) und Live-Statistik werden in Preferences gespeichert.
  // - Warnungen (Wind/Reibung/Temperatur) werden als Safety-WARN gesammelt (GETWARN).
  LoadMonitorConfigPointers lcfg;
  lcfg = LoadMonitorConfigPointers{};

  lcfg.rampDistDeg = &g_rampDistDeg;
  lcfg.calIgnoreRampDeg = &g_calIgnoreRampDeg;
  lcfg.statMinMoveDeg = &g_statMinMoveDeg;
  lcfg.accIgnoreRampDeg = &g_accIgnoreRampDeg;

  lcfg.coldTempDegC = &g_coldTempDegC;
  lcfg.coldExtraDragPct = &g_coldExtraDragPct;

  lcfg.tempWarnAmbientC = &g_tempWarnAmbientC;
  lcfg.tempWarnMotorC = &g_tempWarnMotorC;

  lcfg.dragWarnPct = &g_dragWarnPct;
  lcfg.dragWarnBinsPct = &g_dragWarnBinsPct;
  lcfg.dragPersistMoves = &g_dragPersistMoves;

  lcfg.windPeakPct = &g_windPeakPct;
  lcfg.windCoherenceMin = &g_windCoherenceMin;

  // Preferences sind optional (falls begin() fehlschlaegt):
  // Ohne Preferences laeuft LoadMonitor trotzdem, speichert aber nichts persistent.
  loadMon.begin(g_prefsOk ? &g_prefs : nullptr, &safety, &motion, &temps, lcfg);

  // -------------------------
  // RS485 Proto
  // -------------------------
  // RX-Puffer bewusst vor begin() vergroessern, damit der UART-Treiber
  // eingehende Bursts sicher zwischenlagern kann.
  Serial1.setRxBufferSize(RS485_RX_BUFFER_BYTES);

  rs485.begin(Serial1, RS485_BAUD,
              (gpio_num_t)PIN_RS485_DIR,
              (int)PIN_RS485_RX,
              (int)PIN_RS485_TX,
              g_slaveId,
              RS485_TPRE_US,
              RS485_TPOST_US,
              RS485_RX_BUFFER_BYTES);

  // -------------------------
  // RS485 Dispatcher
  // -------------------------
  Rs485DispatcherConfig dcfg;
  dcfg = Rs485DispatcherConfig{};

  dcfg.ownSlaveId = &g_slaveId;
  dcfg.debug = &g_debug;
  dcfg.logFrames = &g_logRs485Frames;

  dcfg.axisMinDeg01 = &g_axisMinDeg01;
  dcfg.axisMaxDeg01 = &g_axisMaxDeg01;

  // DGOFFSET (rechter Endschalter-Versatz)
  dcfg.dgOffsetDeg01 = &g_dgOffsetDeg01;


// Persistente Parameter (RS485 SET/GET -> Preferences)
dcfg.prefs = g_prefsOk ? &g_prefs : nullptr;

dcfg.homeFastPwmPercent = &g_homeFastPwmPercent;
dcfg.homeBackoff        = &g_homeBackoff;
dcfg.homeReturnToZero   = &g_homeReturnToZero;
dcfg.homeTimeoutMs      = &g_homeTimeoutMs;

dcfg.posTimeoutMs       = &g_posTimeoutMs;

dcfg.handSpeedPercent   = &g_handSpeedPercent;

dcfg.cmdTimeoutMs       = &g_cmdTimeoutMs;

dcfg.isSoftWarnMv       = &g_isSoftWarnMv;
dcfg.isHardStopMv       = &g_isHardStopMv;

// IS Details (neu: EEPROM + RS485)
dcfg.isGraceMs          = &g_isGraceMs;
dcfg.isHardHoldMs       = &g_isHardHoldMs;
dcfg.isFilterLen        = &g_isFilterLen;

dcfg.arriveTolDeg01     = &g_arriveTolDeg01;

dcfg.rampDistDeg        = &g_rampDistDeg;

dcfg.minPwm             = &g_minPwm;

dcfg.stallTimeoutMs     = &g_stallTimeoutMs;

// Stall (neu: EEPROM + RS485)
dcfg.stallMonitorEnabled = &g_stallMonitorEnabled;
dcfg.minStallPwm         = &g_minStallPwm;
dcfg.stallMinCounts      = &g_stallMinCounts;

// PWM-Max: persistent (Default nach Neustart) + runtime override (SETPWM)
dcfg.pwmMaxAbsRuntime   = &g_pwmMaxAbsCmd;
dcfg.pwmMaxAbsStored    = &g_pwmMaxAbsNv;

// Anemometer-Offset (km/h)
dcfg.anemoOffsetKmh     = &g_anemoOffsetKmh;

// Windrichtung-Offset (Grad)
dcfg.windDirOffsetDeg   = &g_windDirOffsetDeg;

// Antennen-Versatz 1..3 (Grad)
dcfg.antOffset1Deg      = &g_antOffset1Deg;
dcfg.antOffset2Deg      = &g_antOffset2Deg;
dcfg.antOffset3Deg      = &g_antOffset3Deg;
dcfg.angle1Deg          = &g_angle1Deg;
dcfg.angle2Deg          = &g_angle2Deg;
dcfg.angle3Deg          = &g_angle3Deg;

// Wind- & Richtungssensor Enable
dcfg.windEnable         = &g_windEnable;

// Homing/Encoder Zusatz (neu: EEPROM + RS485)
dcfg.homeSeekMinPwmPercent = &g_homeSeekMinPwmPercent;
dcfg.homeExpectedCountsRing = &g_homeExpectedCountsRing;
dcfg.homeExpectedCountsMotor = &g_homeExpectedCountsMotor;
dcfg.encTypeU8 = (uint8_t*)&g_encType; // Enum basiert auf uint8_t

// Neustart-Flags (werden von Rs485Dispatcher gesetzt)
dcfg.restartRequested = &g_restartRequested;
dcfg.restartAtMs      = &g_restartAtMs;




  // Temperatur / LoadMonitor / Kalibrierung
  dcfg.tempWarnAmbientC = &g_tempWarnAmbientC;
  dcfg.tempWarnMotorC   = &g_tempWarnMotorC;
  dcfg.tempSwapSensors  = &g_tempSwapSensors;

  dcfg.calIgnoreRampDeg = &g_calIgnoreRampDeg;
  dcfg.statMinMoveDeg   = &g_statMinMoveDeg;
  dcfg.accIgnoreRampDeg = &g_accIgnoreRampDeg;

  dcfg.coldTempDegC     = &g_coldTempDegC;
  dcfg.coldExtraDragPct = &g_coldExtraDragPct;

  dcfg.dragWarnPct      = &g_dragWarnPct;
  dcfg.dragWarnBinsPct  = &g_dragWarnBinsPct;
  dcfg.dragPersistMoves = &g_dragPersistMoves;

  dcfg.windPeakPct      = &g_windPeakPct;
  dcfg.windCoherenceMin = &g_windCoherenceMin;
  dcfg.homingKickTries = &g_homingKickTries;
  dcfg.homingKickNextMs = &g_homingKickNextMs;
  dcfg.homingKickSpacingMs = &g_homingKickSpacingMs;
  dcfg.homingKickMaxTries = &g_homingKickMaxTries;

  rs485Dispatcher.begin(&rs485, &board, &safety, &homing, &motion, &loadMon, &temps, dcfg);

  board.setLed(false);
  g_ledState = false;
  g_ledLastToggleMs = millis();

  if (g_debug) {
    Serial.println("Rotor ready (SETPOSDG + Rampe in Grad + GETPOSDG-Deadman optional)");
    Serial.print("[CFG] handSpeedPercent=");  Serial.println(g_handSpeedPercent, 1);
    Serial.print("[CFG] homeFast/Backoff=%=");
    Serial.print(g_homeFastPwmPercent, 1); Serial.print("/");
    Serial.println(g_homeBackoff, 1);
    Serial.print("[CFG] posPwmMaxAbs(applied/cmd)="); Serial.print(g_pwmMaxAbs, 1); Serial.print("/"); Serial.println(g_pwmMaxAbsCmd, 1);
    Serial.print("[CFG] rampDistDeg=");       Serial.println(g_rampDistDeg, 2);
    Serial.print("[CFG] minPwm=");            Serial.println(g_minPwm, 1);
    Serial.print("[CFG] minStallPwm=");       Serial.println(g_minStallPwm, 1);
    Serial.print("[CFG] cmdTimeoutMs=");      Serial.println(g_cmdTimeoutMs);

    Serial.print("[CFG] endstopDebounceMs="); Serial.println(g_endstopDebounceMs);

    Serial.print("[CFG] IS enabled=");        Serial.println(g_currentMonitorEnabled ? 1 : 0);
    Serial.print("[CFG] IS softWarnMv=");     Serial.println(g_isSoftWarnMv);
    Serial.print("[CFG] IS hardStopMv=");     Serial.println(g_isHardStopMv);
    Serial.print("[CFG] IS graceMs=");        Serial.println(g_isGraceMs);
    Serial.print("[CFG] IS hardHoldMs=");     Serial.println(g_isHardHoldMs);
    Serial.print("[CFG] IS filterLen=");      Serial.println(g_isFilterLen);
    Serial.print("[CFG] IS sampleIntervalMs="); Serial.println(g_isSampleIntervalMs);

    Serial.print("[CFG] IS autoCal=");        Serial.println(g_isAutoCalEnabled ? 1 : 0);
    Serial.print("[CFG] IS autoOffsetMv IS1/IS2=");
    Serial.print(g_isAutoOffset1Mv); Serial.print("/"); Serial.println(g_isAutoOffset2Mv);
    Serial.print("[CFG] IS trimMv IS1/IS2=");
    Serial.print(g_isTrim1Mv); Serial.print("/"); Serial.println(g_isTrim2Mv);
    Serial.print("[CFG] IS appliedOffsetMv IS1/IS2=");
    Serial.print(safetyCfg.isOffset1Mv); Serial.print("/"); Serial.println(safetyCfg.isOffset2Mv);

    Serial.print("[CFG] Stall enabled=");     Serial.println(g_stallMonitorEnabled ? 1 : 0);
    Serial.print("[CFG] Stall timeoutMs=");   Serial.println(g_stallTimeoutMs);
    Serial.print("[CFG] Stall minCounts=");   Serial.println(g_stallMinCounts);


    // Neue Debug-Ausgabe: bewusst in 2 Zeilen halten (die CFG-Zeile ist bereits lang)
    Serial.print("[TEMP] devs="); Serial.print(temps.getDeviceCount());
    Serial.print(" ambC="); Serial.print(((float)temps.getAmbientScaled100())/100.0f, 2);
    Serial.print(" motC="); Serial.print(((float)temps.getMotorScaled100())/100.0f, 2);
    Serial.print(" motEn="); Serial.print(g_motorTempSensorEnabled ? 1 : 0);
    Serial.print(" swap="); Serial.println(g_tempSwapSensors ? 1 : 0);

    Serial.print("[LOAD] ramp="); Serial.print(g_rampDistDeg, 2);
    Serial.print(" calIg="); Serial.print(g_calIgnoreRampDeg, 1);
    Serial.print(" accIg="); Serial.print(g_accIgnoreRampDeg, 1);
    Serial.print(" statMinDeg="); Serial.print(g_statMinMoveDeg, 0);
    Serial.print(" dragWarn%="); Serial.print(g_dragWarnPct, 1);
    Serial.print(" dragBins%="); Serial.print(g_dragWarnBinsPct, 1);
    Serial.print(" dragN="); Serial.print(g_dragPersistMoves);
    Serial.print(" windPeak%="); Serial.print(g_windPeakPct, 1);
    Serial.print(" windCoh%="); Serial.print(g_windCoherenceMin, 1);
    Serial.print(" coldT="); Serial.print(g_coldTempDegC, 1);
    Serial.print(" cold+%="); Serial.println(g_coldExtraDragPct, 1);
  }
}

// ============================================================================

// ============================================================================
// PWM-Max Laufzeit-Rampe (SETPWM waehrend der Fahrt)
// ============================================================================
// Joerg: Wenn du per RS485 (SETPWM) die Geschwindigkeit waehrend einer Bewegung aenderst,
// darf die PWM nicht springen. Deshalb gibt es zwei Werte:
// - g_pwmMaxAbsCmd : Sollwert (kommt direkt aus SETPWM/SETMAXPWM)
// - g_pwmMaxAbs    : geglaetteter Istwert, den MotionController wirklich nutzt
//
// Dadurch wirkt eine neue Geschwindigkeit immer weich (auch im Cruise),
// ohne die Feinphase (Creep) zu beeinflussen.
static float applySlewPercent(float target, float current, uint32_t dtMs, float slewPerSec) {
  if (!isfinite(target)) target = current;
  if (!isfinite(current)) current = target;

  if (dtMs == 0) return current;
  if (!isfinite(slewPerSec) || slewPerSec <= 0.0f) return current;

  const float maxStep = slewPerSec * ((float)dtMs / 1000.0f);
  float delta = target - current;

  if (delta > maxStep) delta = maxStep;
  if (delta < -maxStep) delta = -maxStep;

  return current + delta;
}

static void updatePwmMaxApplied(uint32_t dtMs) {
  // Slew-Rate fuer SETPWM/SETMAXPWM-Aenderungen:
  // 10%/s => 50->90 braucht ~4s, 90->50 braucht ~4s (deutlich sichtbar, kein Ruck).
  const float slewPerSec = 10.0f;

  float target = g_pwmMaxAbsCmd;
  if (!isfinite(target)) target = g_pwmMaxAbs;

  // Grenzen 0..100%
  if (target < 0.0f) target = 0.0f;
  if (target > 100.0f) target = 100.0f;

  // Max darf nicht unter Mindest-PWM fallen, sonst laufen kurze Bewegungen (und Feinphase) nicht sauber.
  if (target < g_minPwm) target = g_minPwm;

  g_pwmMaxAbs = applySlewPercent(target, g_pwmMaxAbs, dtMs, slewPerSec);
}

// ============================================================================
// Encoder-Bereichs-Offset zur Laufzeit uebernehmen
// ============================================================================
// Der DGOFFSET kann per RS485 zur Laufzeit geaendert werden.
// Damit er ohne Neustart wirkt, uebernehmen wir ihn hier in die EncoderAxis.
//
// Hinweis:
// - Diese Anpassung skaliert Counts<->Deg01 leicht um.
// - Wenn man sie waehrend der Bewegung aendert, "springt" die angezeigte Gradposition
//   entsprechend leicht. Sinnvoll ist daher: aendern im Stillstand.
static void updateEncoderRangeOffsetApplied() {
  static int32_t s_lastOff = INT32_MIN;
  if (s_lastOff == g_dgOffsetDeg01) return;
  s_lastOff = g_dgOffsetDeg01;

  encoder.setRangeDegOffsetDeg01(g_dgOffsetDeg01);

  if (g_debug) {
    Serial.print("[CFG] DGOFFSET Deg01=");
    Serial.print(g_dgOffsetDeg01);
    Serial.print(" (=");
    Serial.print((float)g_dgOffsetDeg01 / 100.0f, 2);
    Serial.println("deg)");
  }
}

// Loop
// ============================================================================
void loop() {
  uint32_t nowMs = millis();

  // --------------------------------------------------------------------------
  // Neustart-Handling (durch RS485 SETENCTYPE/SETENCC.. angefordert)
  // --------------------------------------------------------------------------
  if (g_restartRequested) {
    // signed Differenz, damit millis()-Überlauf robust ist
    if ((int32_t)(nowMs - g_restartAtMs) >= 0) {
      if (g_debug) {
        Serial.println("[SYS] Neustart angefordert (EncoderType/EncoderCounts geaendert)");
      }
      // Kleiner Delay, damit Serial/RS485 noch sauber flushen kann.
      delay(50);
      ESP.restart();
      return;
    }
  }

  static uint32_t lastLoopMs = 0;
  uint32_t dtMs = (lastLoopMs == 0) ? 0 : (nowMs - lastLoopMs);
  lastLoopMs = nowMs;

  // RS485 zuerst, damit externe Kommandos auch dann schnell bedient werden,
  // wenn im Hintergrund andere Tasks laufen.
  rs485Dispatcher.update(nowMs);
  updatePwmMaxApplied(dtMs);
  updateEncoderRangeOffsetApplied();

  // Homing
  homing.update(nowMs);
  rs485Dispatcher.updateHomingKickRetry(nowMs);

  // ------------------------------------------------------------
  // Homing-Fehler sichtbar machen (sonst wuerde Homing einfach "still" stehen bleiben)
  // ------------------------------------------------------------
  // Wenn der HomingController in HOME_ERROR geht (z.B. Segment-Timeout), wird der Motor intern gestoppt.
  // Damit der Master das auch ueber GETERR/ERR-Broadcast sieht, latchen wir hier einen Safety-Fehler.
  // Wichtig:
  // - Wir ueberschreiben KEINEN bereits existierenden Safety-Fault (z.B. SE_STALL/SE_TIMEOUT).
  // - Quittierung erfolgt (wie gewuenscht) nur ueber SETREF (Rs485Dispatcher->clearFault()).
  static bool s_homeFailLatched = false;
  if (homing.getState() == HOME_ERROR) {
    if (!safety.isFault() && !s_homeFailLatched) {
      safety.triggerEmergencyStop(SE_HOME_FAIL);
      s_homeFailLatched = true;
    }
  } else {
    // Sobald Homing wieder laeuft/idle ist, Marker loesen (damit ein neuer HOME_ERROR wieder gelatched wird)
    s_homeFailLatched = false;
  }

  // Homing-Status nach dem Update (wichtig, weil sich der Zustand in homing.update() aendern kann)
  const bool homingActive = homing.isActive();

  // Kalibrierfahrt (SETCAL) laeuft intern und darf NICHT am RS485-Deadman scheitern.
  const bool calActive = loadMon.isCalibrationRunning();

  // Im Homing ist es NORMAL, dass ein Endschalter aktiv wird (wir fahren ihn absichtlich an).
  // Deshalb duerfen wir waehrend Homing KEINEN Endstop-Fehler latchen, sondern nur clampen.
  // Ausserhalb Homing bleibt Endstop-Latching aktiv.
  static bool s_lastHomingForEndstopLatch = false;
  if (homingActive != s_lastHomingForEndstopLatch) {
    safety.setLatchErrorOnEndstop(!homingActive);
    s_lastHomingForEndstopLatch = homingActive;
  }

  // ------------------------------------------------------------
  // Endschalter-Ueberfahren im Homing (nur in ganz bestimmten States)
  // ------------------------------------------------------------
  // Hintergrund:
  // - Normalerweise klemmt Safety die PWM auf 0, sobald ein Endschalter aktiv ist
  //   und in diese Richtung gefahren werden soll.
  // - Fuer Phase A (MIN-Endschalter) wollen wir nach dem ersten Ausloesen noch
  //   ein kleines Stueck ueberfahren, um weich abzubremsen.
  // - Daher erlauben wir das Ueberfahren NUR im Overrun-State.
  const HomingState hs = homing.getState();
  const bool allowMinOverrun = (hs == HOME_SEEK_MIN_OVERRUN);
  const bool allowMaxOverrun = false; // aktuell nicht genutzt
  safety.setAllowPushIntoEndMin(allowMinOverrun);
  safety.setAllowPushIntoEndMax(allowMaxOverrun);

  // Umkehrspiel nach Homing uebernehmen (einmalig beim Uebergang auf referenced)
  static bool s_lastRefState = false;
  bool refNow = homing.isReferenced();
  if (refNow && !s_lastRefState) {
    // Wichtig:
    // - ENCTYPE_MOTOR_AXIS: Umkehrspiel wird beim Homing ermittelt und muss in der Positionsregelung
    //   als Ziel-Offset beim Richtungswechsel genutzt werden.
    // - ENCTYPE_RING_OUTPUT: Encoder sitzt auf der Ausgangsachse (Ring) und liefert echte Position.
    //   Hier darf KEIN Umkehrspiel-Offset angewendet werden -> backlash immer 0.
    if (g_encType == ENCTYPE_MOTOR_AXIS) {
      g_backlashDeg01 = homing.getBacklashDeg01();
    } else {
      g_backlashDeg01 = 0;
    }

    if (g_debug) {
      Serial.print("[CFG] Homing done | encType=");
      Serial.print((g_encType == ENCTYPE_MOTOR_AXIS) ? "MOTOR_AXIS" : "RING_OUTPUT");

      Serial.print(" | CPRlearned=");
      Serial.print(homing.getCountsPerRevLearned());
      Serial.print(" | CPRactual=");
      Serial.print(encoder.getCountsPerRevActual());

      if (g_encType == ENCTYPE_MOTOR_AXIS) {
        Serial.print(" | backlashCounts=");
        Serial.print(homing.getBacklashCounts());
        Serial.print(" | backlashDeg01=");
        Serial.println(g_backlashDeg01);
      } else {
        // Ringencoder: Z-Statistik ausgeben (Counts zwischen den Z-Impulsen)
        const EncoderZStats zs = encoder.getZStats();
        Serial.print(" | Zcnt=");
        Serial.print(zs.zCount);
        Serial.print(" dzSteps=");
        Serial.print(zs.dzSteps);
        Serial.print(" dzUs=");
        Serial.print(zs.dzUs);
        Serial.print(" zErr=");
        Serial.print(zs.zErrSteps);
        Serial.print(" corrOff=");
        Serial.println(zs.corrOffsetSteps);
      }
    }
  }
  s_lastRefState = refNow;

  // Motion (ohne Safety)
  // Hinweis: Waehrend Homing ist motion.update() zwar aktiv, aber die PWM kommt aus dem HomingController.
  const float desiredDutyMotion = motion.update(nowMs, dtMs);

  // ------------------------------------------------------------
  // Positionsfahrt-Timeout -> echter Fehler (gelatched) + ERR Broadcast
  // ------------------------------------------------------------
  // Wenn die Positionsfahrt laenger als g_posTimeoutMs dauert, setzt der
  // MotionController ein Event-Flag. Wir wandeln das hier in einen Safety-Fault
  // um, damit:
  // - der Motor wie bei allen anderen Fehlern stoppt
  // - GETERR den Fehler liefert
  // - die asynchrone ERR:<id> Meldung als Broadcast rausgeht
  if (motion.consumePosTimeoutEvent()) {
    safety.triggerEmergencyStop(SE_POS_TIMEOUT);
  }

  // Fuer Safety ist entscheidend, welche PWM tatsaechlich "angefordert" wird:
  // - Im Homing setzt HomingController die PWM direkt am Motor -> dafuer nehmen wir motor.getLastDutySigned().
  // - In der Positionsfahrt (SETPOSDG) kommt die PWM aus dem MotionController -> desiredDutyMotion.
  float desiredDutyForSafety = homingActive ? motor.getLastDutySigned() : desiredDutyMotion;

  // ------------------------------------------------------------
  // Mindest-PWM (g_minPwm) als harter Untergrenzwert
  //
  // Anforderung von Joerg:
  // - Sobald Bewegung gefordert ist (Duty != 0), darf die PWM niemals
  //   unter g_minPwm fallen - egal ob Homing, Positionsfahrt, Rampen etc.
  // - 0 bleibt 0 (Stop ist weiterhin moeglich).
  //
  // Hintergrund:
  // - Beim Homing (RIGHT FAST -> END_MAX) wird die PWM am Ende der Strecke
  //   herunter gerampt. Wenn dieser Wert unter g_minPwm faellt, kann es
  //   passieren, dass der Endschalter nicht mehr sicher gedrueckt wird
  //   und die Stall-Erkennung anspringt.
  // ------------------------------------------------------------
  if (fabsf(desiredDutyForSafety) > 0.01f) {
    const float absD = fabsf(desiredDutyForSafety);
    if (absD < g_minPwm) {
      desiredDutyForSafety = (desiredDutyForSafety > 0.0f) ? g_minPwm : -g_minPwm;
    }
  }

  // Deadman / Keepalive:
  // Waehrend Homing oder Positionsfahrt (RS485-gefuehrt) muss innerhalb g_cmdTimeoutMs
  // mindestens ein GUELTIGES Kommando vom Master empfangen werden, sonst Fault (SE_TIMEOUT).
  // Hinweis: Rs485Dispatcher setzt lastMotionCmdMs bei JEDEM gueltigen Master-Frame (auch GETIS etc.).
  const bool rsMotionActive = (homingActive || motion.isPosActive());
  uint32_t lastMotionCmdMs = rs485Dispatcher.getLastGetPosCmdMs();
  if (lastMotionCmdMs == 0) lastMotionCmdMs = nowMs;
  // Waehrend interner Kalibrierfahrt setzen wir das Keepalive auf "jetzt",
  // damit keine SE_TIMEOUT kommt, obwohl der Master nicht zyklisch sendet.
  if (calActive) lastMotionCmdMs = nowMs;

  // Safety (Strom + Stall + Endstops + Deadman)
  const long safetyEncCounts = encoder.getCountsRaw(); // Stall-Check bewusst RAW, damit Z-Korrektur/Offsets keinen Fake-Fortschritt erzeugen
  // Joerg: Stall-Erkennung darf waehrend STOP-/Richtungswechsel-Bremssequenzen nicht ausloesen.
  // Sonst kann es passieren, dass bei auslaufender PWM noch > Stall-Schwelle ist,
  // aber nur wenige Counts kommen (gewollt), und trotzdem SE_STALL gelatcht wird.
  const float safeDuty = safety.update(nowMs, desiredDutyForSafety, rsMotionActive, lastMotionCmdMs, safetyEncCounts);

  // Motoransteuerung (Safety wirkt jetzt auch im Homing!)
  if (safety.isFault()) {
    // Fehler: Motor aus und Homing sofort abbrechen, damit Homing nicht in der naechsten Loop wieder PWM setzt.
    motor.stopPwm();
    if (homingActive) {
      homing.abort();
    }
    // Positionsfahrt sicher stoppen (damit nach Quittierung nicht "weiter gefahren" wird)
    motion.commandStopSoft();
    // Laufende Live-Statistik / Kalibrierung abbrechen
    loadMon.abortLiveMoveTracking();
    if (loadMon.isCalibrationRunning()) {
      loadMon.abortCalibration(nowMs);
    }
  } else {
    // Kein Fehler:
// - Wenn die Feinphase eine aktive Bremse anfordert: beide Ausgaenge HIGH (dynamische Bremse)
// - Sonst: normales PWM-Verhalten
const bool wantBrake = (!homingActive) && motion.isMotorBrakeRequested();

if (wantBrake) {
  motor.enable(true);
  motor.brake(true);
} else {
  // Bremse ggf. loesen, bevor wir wieder normal PWM/STOP machen
  if (motor.isBraking()) {
    motor.brake(false);
  }

  // PWM anwenden (im Homing ist safeDuty die Homing-PWM; in Motion die geregelte PWM)
  if (fabsf(safeDuty) < 0.01f) {
    motor.stopPwm();
  } else {
    motor.enable(true);

    // ------------------------------------------------------------
    // Harte Mindest-PWM (Joerg):
    // - Sobald Bewegung gefordert ist (Duty != 0), niemals unter g_minPwm.
    // - 0 bleibt 0 (STOP bleibt moeglich).
    //
    // Dieser Clamp ist absichtlich direkt VOR der Motoransteuerung, damit
    // wirklich kein Pfad (Homing-Rampen, Positionsfahrt, Slew, etc.)
    // einen kleineren Duty ausgeben kann.
    // ------------------------------------------------------------
    float outDuty = safeDuty;
    const float absOut = fabsf(outDuty);
    if ((absOut > 0.01f) && (absOut < g_minPwm)) {
      outDuty = (outDuty > 0.0f) ? g_minPwm : -g_minPwm;
    }

    motor.setDutySigned(outDuty);
  }
}
  }


  // LoadMonitor: Statistik / Wind / Temperaturwarnungen aktualisieren
  loadMon.update(nowMs);

  // Zweiter RS485-Durchlauf pro loop():
  // Dadurch wird die maximale Antwortlatenz weiter reduziert, weil auch nach
  // Motion/Safety/LoadMonitor ein moeglicherweise bereits volles RX-Queue-Fenster
  // noch im selben loop()-Durchlauf abgearbeitet wird.
  rs485Dispatcher.update(nowMs);

  // ERR Broadcast beim Fault-Eintritt
  bool faultNow = safety.isFault();
  if (faultNow && !g_faultPrev) {
    uint8_t ec = safety.getErrorCode();
    if ((nowMs - g_lastErrBroadcastMs) > 200) {
      g_lastErrBroadcastMs = nowMs;
      if (ec != 0) rs485Dispatcher.sendErrBroadcast(ec);
    }
  }
  g_faultPrev = faultNow;

  // LED
  bool moving = isMovingByCommand(homingActive, safeDuty);
  updateLbLed(nowMs, moving, faultNow);

  // Debug: schneller und nur wenn wirklich Bewegung/Regelung aktiv ist
  if (g_debug) {
    static uint32_t lastMs = 0;
    if ((nowMs - lastMs) >= 50) {
      lastMs = nowMs;

      int32_t curDeg01 = 0;
      encoder.getPositionDeg01(curDeg01);

      MotionDebugSnapshot ms = motion.getDebugSnapshot(curDeg01);

      if (faultNow || homingActive || ms.posActive || fabsf(safeDuty) > 0.01f) {
        Serial.print("[DBG] fault=");
        Serial.print(faultNow ? 1 : 0);
        Serial.print(" err=");
        Serial.print((int)safety.getErrorCode());

        Serial.print(" | pos=");
        Serial.print(ms.posActive ? 1 : 0);
        Serial.print(" hom=");
        Serial.print(homingActive ? 1 : 0);
        Serial.print(" ref=");
        Serial.print(homing.isReferenced() ? 1 : 0);

        Serial.print(" | cur=");
        Serial.print(ms.curDeg01);
        Serial.print(" tgt=");
        Serial.print(ms.tgtDeg01);

        // Debug-Fehleranzeige: negativ bedeutet "über Soll" (tgt - cur).
        // Bei einseitigem Ankommen (cur >= tgt) steht am Ende damit immer ein Minus oder 0.
        const int32_t eDbgDeg01 = (int32_t)ms.tgtDeg01 - (int32_t)ms.curDeg01;

        Serial.print(" e=");
        Serial.print(eDbgDeg01);
// Modus/Phase fuer Debug (aus Zustaenden abgeleitet)
        const bool brkReq = motion.isMotorBrakeRequested();
        const bool brkAct = motor.isBraking();
        const bool fineExact = motion.isFineExactMode();
        const uint8_t fineRtry = motion.getFineExactRetryCount();
        const int32_t absErr01 = (ms.errDeg01 < 0) ? -ms.errDeg01 : ms.errDeg01;
        const bool inFine = (g_fineWindowDeg01 > 0) && (absErr01 <= g_fineWindowDeg01) && ms.posActive;
        const char* modeStr = (!ms.posActive) ? "IDLE" : (brkReq ? "FBRK" : (inFine ? "FINE" : "COARSE"));

        Serial.print(" | m=");
        Serial.print(modeStr);
        Serial.print(" brkReq=");
        Serial.print(brkReq ? 1 : 0);
        Serial.print(" brkAct=");
        Serial.print(brkAct ? 1 : 0);
        Serial.print(" ex=");
        Serial.print(fineExact ? 1 : 0);
        Serial.print(" rtry=");
        Serial.print((int)fineRtry);
        Serial.print(" win=");
        Serial.print(g_fineWindowDeg01);

        Serial.print(" | v=");
        Serial.print(ms.speedMeasDegPerSec, 2);

        Serial.print(" | dDes=");
        Serial.print(desiredDutyForSafety, 1);
        Serial.print(" dSafe=");
        Serial.print(safeDuty, 1);

        Serial.print(" | mDuty=");
        Serial.print(motor.getLastDutySigned(), 1);


        // Stromwerte (IS) in mV:
        // - avg/raw sind NACH Offset-Abzug (diese Werte zaehlen fuer Soft/Hard-Limits)
        // - (adc) ist der Rohwert VOR Offset-Abzug (nur Diagnose)
        SafetyIsSnapshot isSnap = safety.getIsSnapshot();
        Serial.print(" | IS1=");
        Serial.print(isSnap.avg1);
        Serial.print("/");
        Serial.print(isSnap.raw1);
        Serial.print("(");
        Serial.print(isSnap.adc1);
        Serial.print(")");
        Serial.print(" IS2=");
        Serial.print(isSnap.avg2);
        Serial.print("/");
        Serial.print(isSnap.raw2);
        Serial.print("(");
        Serial.print(isSnap.adc2);
        Serial.print(")");

        Serial.print(" | maxC=");
        Serial.print(encoder.getCountsPerRevActual());

        // Encoder-/Backlash-/Z-Diagnose
        if (encoder.getEncoderType() == ENCTYPE_MOTOR_AXIS) {
          Serial.print(" | BLcfg=");
          Serial.print(ms.backlashCfgDeg01);
          Serial.print(" BLap=");
          Serial.print(ms.backlashAppliedDeg01);
          Serial.print(" lastDir=");
          Serial.print(ms.lastMoveDirNonZero);
          Serial.print(" dir=");
          Serial.print(ms.moveDir);
        } else {
          const EncoderZStats zs = encoder.getZStats();
          Serial.print(" | Zcnt=");
          Serial.print(zs.zCount);
          Serial.print(" dz=");
          Serial.print(zs.dzSteps);
          Serial.print(" zErr=");
          Serial.print(zs.zErrSteps);
          Serial.print(" off=");
          Serial.print(zs.corrOffsetSteps);
        }

        Serial.println();
      }
    }
  }
}
