#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <PZEM004Tv30.h>
#include <HardwareSerial.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <time.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include "secret.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// SSID and PASS
const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASSWORD;

const char* VERSION_URL = "https://raw.githubusercontent.com/rxceed/power-monitoring-system-esp32-firmware/refs/heads/main/version.txt";
const char* FIRMWARE_BASE_URL = "https://github.com/rxceed/power-monitoring-system-esp32-firmware/releases/download/v";

const int FIRMWARE_VERSION = 3;

// MQTT Broker
const char* mqtt_server = MQTT_BROKER;

// Create WiFi and MQTT clients
WiFiClient espClient;
PubSubClient client(espClient);

// Heartbeat interval
long lastMsg = 0;
const long interval = 2000; // 2 seconds

// MQTT topics
const char* voltage_topic = "esp32/voltage";
const char* current_topic = "esp32/current";
const char* power_topic   = "esp32/power";
const char* energy_topic  = "esp32/energy";
const char* frequency_topic = "esp32/frequency";
const char* pf_topic      = "esp32/powerFactor";


// Define a pin
#define LED 2 
#define PZEM_DEFAULT_ADDR 0x01
#define RX_PIN 16
#define TX_PIN 17
#define RELAY_PIN 13

HardwareSerial espSerial(2); // RX, TX
PZEM004Tv30 pzem(&espSerial, RX_PIN, TX_PIN, PZEM_DEFAULT_ADDR);

typedef struct
{
  float voltage;
  float current;
  float power;
  float energy;
  float frequency;
  float pf;
} PZEM_task_params;

typedef struct 
{
  float catboost;
  float rnn;
} classification_confidence;


//PZEM args
PZEM_task_params PZEMParams{
  .voltage = 0,
  .current = 0,
  .power = 0,
  .energy = 0,
  .frequency = 0,
  .pf = 0,
};

classification_confidence confidence{
  .catboost = 1,
  .rnn = 1
};

static char relayControl = 0;

SemaphoreHandle_t pzemMutex;

// Task handles
TaskHandle_t ReadPZEMTaskHandle = NULL;

// MQTT Subscribe callback function
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print(message);
  Serial.printf("\n");
  if(!strcmp(topic, "inference/confidence_catboost"))
  {
    confidence.catboost = message.toFloat();
  }
  else if(!strcmp(topic, "inference/confidence_rnn"))
  {
    confidence.rnn = message.toFloat();
  }
  else if(!strcmp(topic, "esp32/relay/control"))
  {
    relayControl = 1;
  }
}

// Wifi connection function
void setup_wifi(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(100));
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_MODE_STA);

  WiFi.begin(ssid, password);
  WiFi.setAutoReconnect(true);
  while(1)
  {
    if(WiFi.status() != WL_CONNECTED) 
    {
      Serial.print(".");
      if(WiFi.status() == WL_CONNECTED)
      {
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// MQTT reconnect function
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "N1663RK1LL3R";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe("inference/confidence_catboost");
      client.subscribe("inference/confidence_rnn");
      client.subscribe("esp32/relay/control");
      // Once connected, publish an announcement...
      client.publish("esp32/status", "NYAMBUNG CAK!");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 0.5 seconds");
      // Wait 5 seconds before retrying
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
}

// Task function to read PZEM data
void ReadPZEMTask(void *pvParameters) {
  PZEM_task_params *params = (PZEM_task_params*) pvParameters;
  while(1)
  {
    params->voltage = pzem.voltage();
    vTaskDelay(10);
    params->current = pzem.current();
    vTaskDelay(10);
    params->power = pzem.power();
    vTaskDelay(10);
    params->energy = pzem.energy();
    vTaskDelay(10);
    params->frequency = pzem.frequency();
    vTaskDelay(10);
    params->pf = pzem.pf();

    Serial.printf("%.2f V; %.2f A; %.2f W; %.2f Wh; %.2f Hz; %.2f\n", 
      params->voltage, params->current, params->power, params->energy, params->frequency, params->pf );
    vTaskDelay(pdMS_TO_TICKS(200)); // Delay for 2 seconds
  }
}

// Task function to handle MQTT communication
void MQTTTask(void *pvParameters) {
  while(WiFi.status() != WL_CONNECTED)
  {
    Serial.println("MQTT: Waiting for wifi to connect...");
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  client.setKeepAlive(5);
  PZEM_task_params *params = (PZEM_task_params*) pvParameters;
  while(1) 
  {
    printf("MQTT STATUS: %d\n", client.connected());
    if (!client.connected()) {
      reconnect();
    }
    client.loop();

    char voltageStr[16];
    dtostrf(params->voltage, 1, 2, voltageStr);
    client.publish(voltage_topic, voltageStr);

    vTaskDelay(10);

    char currentStr[16];
    dtostrf(params->current, 1, 2, currentStr);
    client.publish(current_topic, currentStr);

    vTaskDelay(10);

    char powerStr[16];
    dtostrf(params->power, 1, 2, powerStr);
    client.publish(power_topic, powerStr);

    vTaskDelay(10);

    char energyStr[16];
    dtostrf(params->energy, 1, 2, energyStr);
    client.publish(energy_topic, energyStr);

    vTaskDelay(10);

    char frequencyStr[16];
    dtostrf(params->frequency, 1, 2, frequencyStr);
    client.publish(frequency_topic, frequencyStr);

    vTaskDelay(10);

    char pfStr[16];
    dtostrf(params->pf, 1, 2, pfStr);
    client.publish(pf_topic, pfStr);
    printf("Published all!\n");
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void relayTask(void *pvParameters) {
  long long timer = millis();
  long long timerUnknownDevice = 0;
  while (true) {
    if(relayControl == 0)
    {
      if(confidence.catboost < 0.4 && confidence.rnn < 0.5)
      {
        if(timerUnknownDevice >= 5000)
        {
          digitalWrite(RELAY_PIN, 0);
          printf("Unknown Device Detected!\n");
        }
        timerUnknownDevice = millis() - timer;
      }
      else if(confidence.catboost < 0.3)
      {
        if(timerUnknownDevice >= 5000)
        {
          digitalWrite(RELAY_PIN, 0);
          printf("Unknown Device Detected!\n");
        }
        timerUnknownDevice = millis() - timer;
      }
      else if(confidence.rnn < 0.4)
      {
        if(timerUnknownDevice >= 5000)
        {
          digitalWrite(RELAY_PIN, 0);
          printf("Unknown Device Detected!\n");
        }
        timerUnknownDevice = millis() - timer;
      }
      else if(confidence.catboost >= 0.4 && confidence.rnn >= 0.5)
      {
        timerUnknownDevice = 0;
        timer = millis();
      }
    }
    else if(relayControl == 1)
    {
      digitalWrite(RELAY_PIN, 1);
      relayControl = 0;
      timer = millis();
      timerUnknownDevice = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// Helper function to resolve GitHub redirects manually
String getFinalURL(String url) {
  HTTPClient http;
  WiFiClientSecure client;
  client.setInsecure(); // We only need to grab the header, security is less critical here

  // We need to stop the internal library from following the redirect automatically
  // so we can grab the new URL cleanly.
  http.begin(client, url);
  
  // We want to read the "Location" header which contains the new URL
  const char *headerKeys[] = {"Location"};
  http.collectHeaders(headerKeys, 1);

  int httpCode = http.GET();
  String newUrl = "";

  // Check for Redirect (301 or 302)
  if (httpCode == HTTP_CODE_MOVED_PERMANENTLY || httpCode == HTTP_CODE_FOUND) {
    newUrl = http.header("Location");
    Serial.println("Redirect detected!");
    Serial.println("Original: " + url);
    Serial.println("New:      " + newUrl);
  } 
  else if (httpCode == HTTP_CODE_OK) {
    // If GitHub somehow gave us the file directly (rare)
    newUrl = url; 
  } 
  else {
    Serial.printf("Error resolving URL. HTTP Code: %d\n", httpCode);
  }
  
  http.end();
  return newUrl;
}

void checkFirmwareUpdate(void *pvParameters) {
  while(WiFi.status() != WL_CONNECTED)
  {
    Serial.println("OTA: Waiting for wifi to connect...");
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  Serial.println("Checking for firmware updates...");

  WiFiClientSecure client;
  client.setInsecure(); // Use setCACert(root_ca) for production

  HTTPClient http;
  
  while(1)
  {
    // 1. GET THE VERSION TEXT FILE
    if (!http.begin(client, VERSION_URL)) {
      Serial.println("Cannot connect to version file URL");
      return;
    }

    int httpCode = http.GET();
    if (httpCode != HTTP_CODE_OK) {
      Serial.printf("Version check failed, HTTP Code: %d\n", httpCode);
      http.end();
      return;
    }

    // 2. READ AND PARSE VERSION
    String payload = http.getString();
    http.end();

    payload.trim(); 
    int newVersion = payload.toInt();

    Serial.printf("Current Version: %d, Server Version: %d\n", FIRMWARE_VERSION, newVersion);

    // 3. COMPARE AND UPDATE
    if (newVersion > FIRMWARE_VERSION) {
      Serial.println("New firmware detected! Starting update...");
      
      String firmwareUrl = FIRMWARE_BASE_URL + String(newVersion) + "/firmware.bin";
      String finalUrl = getFinalURL(firmwareUrl);
    
      if (finalUrl == "") {
        Serial.println("Could not get the final URL. Aborting.");
        return;
      }
      Serial.println("Target URL: " + finalUrl);
      t_httpUpdate_return ret = httpUpdate.update(client, finalUrl);

      switch (ret) {
        case HTTP_UPDATE_FAILED:
          Serial.printf("Update Failed. Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
          break;
        case HTTP_UPDATE_NO_UPDATES:
          Serial.println("No updates.");
          break;
        case HTTP_UPDATE_OK:
          Serial.println("Update OK."); // System will restart automatically
          break;
      }
    } else {
      Serial.println("Device is up to date.");
    }
    vTaskDelay(pdMS_TO_TICKS(1000*60*60));
  }
}

void setup() {
  Serial.begin(115200);
  espSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, 1);
  xTaskCreatePinnedToCore(
    setup_wifi,
    "Setup wifi task",
    8192,
    NULL,
    3,
    NULL,
    0
  );
  // Create the ReadPZEMTask pinned to core 1
  xTaskCreatePinnedToCore(
      ReadPZEMTask,          // Task function
      "ReadPZEMTask",       // Name of the task
      4096,                 // Stack size in words
      &PZEMParams,                 // Task input parameter
      1,                    // Priority of the task
      NULL,  // Task handle
      1                     // Core where the task should run
  );
  
  //Create the MQTTTask pinned to core 1
  xTaskCreatePinnedToCore(
       MQTTTask,             // Task function
       "MQTTTask",          // Name of the task
       8192,                // Stack size in words
       &PZEMParams,                // Task input parameter
       2,                   // Priority of the task
       NULL,                // Task handle
       1                    // Core where the task should run
   );

  // Create the relayTask pinned to core 1
  xTaskCreatePinnedToCore(
      relayTask, // Task function
      "RelayTask",               // Name of the task
      2048,                      // Stack size in words
      NULL,                      // Task input parameter
      1,                         // Priority of the task
      NULL,                      // Task handle
      1                          // Core where the task should run
  );

  xTaskCreatePinnedToCore(
    checkFirmwareUpdate,
    "Firmware Update Task",
    16384,
    NULL,
    2,
    NULL,
    0
  );

}

void loop() {

}