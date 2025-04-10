#include <WiFi.h>
// Version 3.3.8
// https://github.com/ESP32Async/AsyncTCP
#include <AsyncTCP.h>
// Version 4.0.1
// https://github.com/karol-brejna-i/RemoteDebug
#include <RemoteDebug.h>   //Debug via Wifi
// Version 3.2.0
// https://github.com/espressif/arduino-esp32
#include <esp_task_wdt.h>  //For Watchdog

#define LED 2

//Debug via WIFI instead of Serial
//Connect a Telnet terminal on port 23
RemoteDebug Debug;

//PINS - GPIO
#define RXD2 26
#define TXD2 27

//Watchdog every 120 secondes / 2 min.
//Reset system when no communication with LINKY during 120s
#define WDT_TIMEOUT 12000

// To swtich from history to standard
// https://support.ecojoko.com/hc/fr/articles/9077627829020-Passage-du-compteur-Linky-en-mode-standard
// https://particulier.edf.fr/fr/accueil/espace-client/contact/demande/formulaire/decret-conso.html#/selection-droit-exerce
#define LINKY_BAUD_STANDARD_MODE 9600
#define LINKY_BAUD_HISTORY_MODE 1200

#define INJECTION_THRESHOLD_FOR_HEATING 3100
#define RELAY_WATER_HEATER 13

esp_task_wdt_config_t twdt_config = {
        .timeout_ms = WDT_TIMEOUT,
        .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1,    // Bitmask of all cores
        .trigger_panic = true,
    };

const char* ssid = "redchicken";          // Your WiFi SSID
const char* password = "wxcvbnpoiuytreza";  // Your WiFi Password

int IdxDataRawLinky = 0;
int IdxBufferLinky = 0;
char DataRawLinky[1000];
char BufferLinky[30];
float IWattTook = 0;  // Instantaneous watt took from the grid
float AvWattTook = 0;   // Average watt took from the grid over 5 min
float IWattSent = 0;   // Instantaneous watt sent to the grid
float AvWattSent = 0;  // Average watt sent to the grid over 5 min
bool LFon = false;
bool relayOn = false;
bool updateReceived = false;

//Internal Timers
unsigned long previousWatchdogMillis;
unsigned long relayOnMillis;
unsigned long relayOffMillis;

// Read LINKY data on serial PORT I1-I2
void readLinky() {
  updateReceived = false;
  if (Serial2.available() > 0) {
    int V = Serial2.read();
    if (V == 2) {  //STX (Start Text)
      for (int i = 0; i < 5; i++) {
        DataRawLinky[IdxDataRawLinky] = '-';
        IdxDataRawLinky = (IdxDataRawLinky + 1) % 1000;
      }
      // digitalWrite(LED, LOW);
    }
    if (V == 3) {  //ETX (End Text)
      // digitalWrite(LED, HIGH);
      // reset watch dog every 3s to avoid reset after 2min
      // Linky data was received in time
      // Group of data should be received every 2s
      if (millis() - previousWatchdogMillis > 3000) {
        esp_task_wdt_reset();
        previousWatchdogMillis = millis();
      }
    }
    // Not ETX or STX
    if (V > 9) { 
      switch (V) {
        case 10:  // Line Feed. Start group
          LFon = true;
          IdxBufferLinky = 0;
          break;
        case 13:       // End group
          if (LFon) {  //OK Start group
            LFon = false;
            int nb_blanc = 0;
            String code = "";
            String val = "";
            // line treatment and decoding
            for (int i = 0; i < IdxBufferLinky; i++) {
              if (BufferLinky[i] == ' ') {
                nb_blanc++;
              }
              if (nb_blanc == 0) {
                code += BufferLinky[i];
              }
              if (nb_blanc == 1) {
                val += BufferLinky[i];
              }
              // Check is ignored
              if (nb_blanc < 2) {
                DataRawLinky[IdxDataRawLinky] = BufferLinky[i];
                IdxDataRawLinky = (IdxDataRawLinky + 1) % 1000;
              }
            }
            DataRawLinky[IdxDataRawLinky] = char(13);
            IdxDataRawLinky = (IdxDataRawLinky + 1) % 1000;
            // Average pulled power over 5 min
            if (code.startsWith("SINSTS")) {
              IWattTook = code.substring(6, 11).toFloat();
              if (AvWattTook == 0) { AvWattTook = IWattTook; }
              AvWattTook = (IWattTook + 149 * AvWattTook) / 150;
              debugI("CONSUMED: %.0f | %.0f", IWattTook, AvWattTook);
              updateReceived = true;
            }
            // Average injected power over 5 min
            if (code.startsWith("SINSTI")) {
              IWattSent = code.substring(6, 11).toFloat();
              if (AvWattSent == 0) { AvWattSent = IWattSent; }
              AvWattSent = (IWattSent + 149 * AvWattSent) / 150;
              debugI("PRODUCTED: %.0f | %.0f", IWattSent, AvWattSent);
              updateReceived = true;
            }
          }
          break;
        default:
          // line data accumulation in buffer
          BufferLinky[IdxBufferLinky] = char(V);
          IdxBufferLinky = (IdxBufferLinky + 1) % 30;
          break;
      }
      // Debug output example
      // (D) SINSTS00000F
      // (D) SINSTI02813J
      if (Debug.isActive(Debug.DEBUG)) {
        Debug.print(char(V));
      }
    }
  }
}

void relayWaterHeater()
{
  // refresh every 2s
  if (updateReceived) {
    unsigned long timeRelayOff = millis() - relayOffMillis;
    unsigned long timeRelayOn = millis() - relayOnMillis;

    // When average injection threshold has been reached, we can start heating
    // Only when relay was off during at least 5 min
    // Only when average watt took from the grid during 5 min was null
    if (IWattSent > INJECTION_THRESHOLD_FOR_HEATING && AvWattTook == 0 && !relayOn && timeRelayOff > 300000) {
      digitalWrite(LED, HIGH);
      digitalWrite(RELAY_WATER_HEATER, HIGH);
      relayOn = true;
      relayOnMillis = millis();
    }

    // Immidiately stop heating when we take power from the grid
    // Stop heating after 3h per day
    if (relayOn && (IWattTook > 0 || timeRelayOn > 10800000)) {
      digitalWrite(LED, LOW);
      digitalWrite(RELAY_WATER_HEATER, LOW);
      relayOn = false;
      relayOffMillis = millis();
    }

    if (relayOn) {
      debugI("Relay ON during %lu s | %lu min", timeRelayOn / 1000, timeRelayOn / 60000);
    } else {
      debugI("Relay OFF during %lu s | %lu min", timeRelayOff / 1000, timeRelayOff / 60000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  //  7-bit Even parity 1 stop bit for Linky
  Serial2.begin(LINKY_BAUD_STANDARD_MODE, SERIAL_7E1, RXD2, TXD2);  
  pinMode(LED, OUTPUT);
  pinMode(RELAY_WATER_HEATER, OUTPUT);

  // Stop heating when ESP32 reset
  digitalWrite(LED, LOW);
  digitalWrite(RELAY_WATER_HEATER, LOW);

  //Watchdog initialisation
  esp_task_wdt_init(&twdt_config); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  Serial.println("Start WiFi connection...");
  WiFi.mode(WIFI_STA);

  esp_task_wdt_reset();

  // WiFi connection is not mandatory
  // It is just for debugging and watching
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi Failed!\n");
    return;
  }
  // Init remote debug
  // Telnet on port 23
  Debug.begin("ESP32", Debug.INFO);  

  Serial.print("Debug connection: telnet ");
  Serial.print(WiFi.localIP());
  Serial.println(" 23");

  //Timers
  previousWatchdogMillis = millis();
  relayOffMillis = millis();
  relayOnMillis = 0;
}

void loop() {
  Debug.handle();
  readLinky();
  //powerInjectedIndication();
  relayWaterHeater();
  esp_task_wdt_reset();
}
