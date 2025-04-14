#include <WiFi.h>
#include <HTTPClient.h>
// Version 3.3.8
// https://github.com/ESP32Async/AsyncTCP
#include <AsyncTCP.h>
// Version 4.0.1
// https://github.com/karol-brejna-i/RemoteDebug
#include <RemoteDebug.h>   //Debug via Wifi
// Version 3.2.0
// https://github.com/espressif/arduino-esp32
#include <esp_task_wdt.h>  //For Watchdog
// Web Server
// Version 3.7.6
// https://github.com/ESP32Async/ESPAsyncWebServer
#include <ESPAsyncWebServer.h>
// Version 1.4.6
// https://github.com/adafruit/DHT-sensor-library
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 22
#define DHTTYPE DHT11

// init sensor temperature and humidity
DHT_Unified dht(DHTPIN, DHTTYPE);

#define LED_RELAY 15
#define LED_1KW 19
#define LED_2KW 5
#define LED_3KW 16
#define LED_4KW 0

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

// GCE API EndPoint
#define HOST_NAME "http://192.168.1.240"
#define PATH_NAME "/api/xdevices.json"      
#define QUERY_RELAY_ON "key=apikey&SetR=11"
#define QUERY_RELAY_OFF "key=apikey&ClearR=11"

esp_task_wdt_config_t twdt_config = {
        .timeout_ms = WDT_TIMEOUT,
        .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1,    // Bitmask of all cores
        .trigger_panic = true,
    };

const char* ssid = "";          // Your WiFi SSID
const char* password = "";  // Your WiFi Password

// HTTP WebServer
AsyncWebServer server(80);

int IdxDataRawLinky = 0;
int IdxBufferLinky = 0;
char DataRawLinky[1000];
char BufferLinky[30];
float IWattTook = 0;  // Instantaneous watt took from the grid
float AvWattTook = 0;   // Average watt took from the grid over 5 min
float IWattSent = 0;   // Instantaneous watt sent to the grid
float AvWattSent = 0;  // Average watt sent to the grid over 5 min
int currentHour = 0; // current hour in the day
bool LFon = false;
bool relayOn = false;
bool updateReceived = false;

//Internal Timers
unsigned long previousWatchdogMillis;
unsigned long relayOnMillis;
unsigned long relayOffMillis;

// DTH values
float temperature = 0;
float humidity = 0;

// Remote relay control on GCE IPX800 V4
// Call example :
// http://192.168.1.240/api/xdevices.json?key=apikey&SetR=11
// http://192.168.1.240/api/xdevices.json?key=apikey&ClearR=11
void remoteRelayControl(bool relayOn) {
  HTTPClient http;
  char query[65];

  if (relayOn) {
    sprintf(query, "%s%s?%s", HOST_NAME, PATH_NAME, QUERY_RELAY_ON);
  } else {
    sprintf(query, "%s%s?%s", HOST_NAME, PATH_NAME, QUERY_RELAY_OFF);
  }
  http.begin(query);
  http.addHeader("Content-Type", "application/json");
  int httpCode = http.GET();

  // httpCode will be negative on error
  if (httpCode > 0) {
    // file found at server
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      debugI("%s",payload);
    } else {
      // HTTP header has been send and Server response header has been handled
      debugI("[HTTP] GET... code: %d\n", httpCode);
    }
  } else {
    debugE("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode));
  }

  http.end();
}

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
            // get day hour in 24H format
            if (code.startsWith("DATEE")) {
              currentHour = code.substring(11, 13).toInt();
              debugI("CURRENT HOUR: %d", currentHour);
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
      // (D) DATEE250411105904>
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
    // Only when current watt took from the grid is null
    // Only during daylight period
    debugD("AvWattSent %0.f > %ul => %d", AvWattSent, INJECTION_THRESHOLD_FOR_HEATING, AvWattSent > INJECTION_THRESHOLD_FOR_HEATING);
    debugD("IWattTook %0.f == 0 => %d", IWattTook, IWattTook == 0);
    debugD("!relayOn => %d", !relayOn);
    debugD("timeRelayOff %lu > 300000 => %d", timeRelayOff, timeRelayOff > 300000);
    debugD("currentHour %d > 8 && currentHour %d < 21 => %d", currentHour, currentHour, currentHour > 8 && currentHour < 21);
    if (AvWattSent > INJECTION_THRESHOLD_FOR_HEATING && IWattTook == 0 && !relayOn && timeRelayOff > 300000 && currentHour > 8 && currentHour < 21) {
      digitalWrite(LED_RELAY, HIGH);
      relayOn = true;
      relayOnMillis = millis();
      remoteRelayControl(relayOn);
    }

    // Immidiately stop heating when we take power from the grid
    // Stop heating after 3h per day
    debugD("IWattTook %0.f > 0 => %d", IWattTook, IWattTook > 0);
    debugD("timeRelayOn %lu > 10800000", timeRelayOn, timeRelayOn > 10800000);
    if (relayOn && (IWattTook > 0 || timeRelayOn > 10800000)) {
      digitalWrite(LED_RELAY, LOW);
      digitalWrite(RELAY_WATER_HEATER, LOW);
      relayOn = false;
      relayOffMillis = millis();
      remoteRelayControl(relayOn);
    }

    if (relayOn) {
      debugI("Relay ON during %lu s | %lu min", timeRelayOn / 1000, timeRelayOn / 60000);
    } else {
      debugI("Relay OFF during %lu s | %lu min", timeRelayOff / 1000, timeRelayOff / 60000);
    }
  }
}

void powerInjectedIndication() {
  if (updateReceived) {

    // turn on
    if (IWattSent > 1000) {
      digitalWrite(LED_1KW, HIGH);
    }
    if (IWattSent > 2000) {
      digitalWrite(LED_2KW, HIGH);
    }
    if (IWattSent > 3000) {
      digitalWrite(LED_3KW, HIGH);
    }
    if (IWattSent > 4000) {
      digitalWrite(LED_4KW, HIGH);
    }

    // turn off
    if (IWattSent < 1000) {
      digitalWrite(LED_1KW, LOW);
    }
    if (IWattSent < 2000) {
      digitalWrite(LED_2KW, LOW);
    }
    if (IWattSent < 3000) {
      digitalWrite(LED_3KW, LOW);
    }
    if (IWattSent < 4000) {
      digitalWrite(LED_4KW, LOW);
    }
  }
}

// Le DHT11 refresh data every 1s
void sensorDTH11Update() {
  if (updateReceived) {
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    debugE("Fail to recieve temperature DTH11 data");
    return;
  } else {
    temperature = event.temperature;
  }
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    debugE("Fail to recieve humidity DTH11 data");
  } else {
    humidity = event.relative_humidity;
  }
  debugI("Humidite %0.2f%", humidity);
  debugI("Temperature %0.2f°C", temperature);
  }
}

// Expose consumption/production data in JSON format for GCE IPX800 Dashboard
void onGetLinkyAndTemperatureInformation(AsyncWebServerRequest *request) {
  char payload[200];
  char payload_size[4];

  sprintf(payload, "{\"consumption\" : \"%0.f\", \"production\" : \"%0.f\", \"temperature\" : \"%0.2f\", \"humidity\" : \"%0.2f\"}", IWattTook, IWattSent, temperature, humidity);
  AsyncWebServerResponse *response = request->beginResponse(200, "application/json; charset=utf-8",  payload);
  response->addHeader("Server","ESP32-Linky");
  response->addHeader("Access-Control-Allow-Methods", "GET");
  response->addHeader("Access-Control-Allow-Origin", "*");
     
  // lower case sent by ESPAsyncWebServer. Resend in upper caser, in case of.
  response->addHeader("Content-Type", "application/json; charset=utf-8");
  sprintf(payload_size, "%d", strlen(payload));
  response->addHeader("Content-Length", payload_size);
  request->send(response);
}

void setup() {
  // sensor t° and humidity startup
  dht.begin();
  Serial.begin(115200);
  //  7-bit Even parity 1 stop bit for Linky
  Serial2.begin(LINKY_BAUD_STANDARD_MODE, SERIAL_7E1, RXD2, TXD2);
  pinMode(LED_RELAY, OUTPUT);
  pinMode(LED_1KW, OUTPUT);
  pinMode(LED_2KW, OUTPUT);
  pinMode(LED_3KW, OUTPUT);
  pinMode(LED_4KW, OUTPUT);
  
  // turn off LED
  digitalWrite(LED_1KW, LOW);
  digitalWrite(LED_2KW, LOW);
  digitalWrite(LED_3KW, LOW);
  digitalWrite(LED_4KW, LOW);

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
    Serial.printf("WiFi Failed!... Restart in 5s\n");
    delay(5000);
    ESP.restart();
  }
  // Init remote debug
  // Telnet on port 23
  Debug.begin("ESP32", Debug.ERROR);  

  Serial.print("Debug connection: telnet ");
  Serial.print(WiFi.localIP());
  Serial.println(" 23");

  if (WiFi.isConnected()) {
    // Expose consumption/production data in JSON format
	  server.on("/linky.json", HTTP_GET, onGetLinkyAndTemperatureInformation);
   
	  // Start web server
    Serial.println("Start WebServer...");
	  server.begin(); 

    // Stop heating when ESP32 reset
    Serial.println("Stop heating");
    digitalWrite(LED_RELAY, LOW);
    remoteRelayControl(false);
  }
  
  //Timers
  previousWatchdogMillis = millis();
  relayOffMillis = millis();
  relayOnMillis = 0;
}

void loop() {
  Debug.handle();
  readLinky();
  sensorDTH11Update();
  powerInjectedIndication();
  relayWaterHeater();
  esp_task_wdt_reset();
}
