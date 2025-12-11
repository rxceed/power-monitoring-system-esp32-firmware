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

const int FIRMWARE_VERSION = 1;

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
#define RELAY_PIN 15

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

//PZEM args
PZEM_task_params PZEMParams{
  .voltage = 0,
  .current = 0,
  .power = 0,
  .energy = 0,
  .frequency = 0,
  .pf = 0,
};

SemaphoreHandle_t pzemMutex;

// Task handles
TaskHandle_t ReadPZEMTaskHandle = NULL;

// MQTT Subscribe callback function
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// Wifi connection function
void setup_wifi() {
  delay(1000);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_MODE_STA);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
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
      // Once connected, publish an announcement...
      client.publish("esp32/status", "NYAMBUNG CAK!");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void subscribeTopics() {
  client.setCallback(callback);
  client.subscribe("niggasLair/status");
}

// Task function to read PZEM data
void ReadPZEMTask(void *pvParameters) {
  PZEM_task_params *params = (PZEM_task_params*) pvParameters;
  while(1)
  {
    params->voltage = pzem.voltage();
    params->current = pzem.current();
    params->power = pzem.power();
    params->energy = pzem.energy();
    params->frequency = pzem.frequency();
    params->pf = pzem.pf();

    Serial.printf("%.2f V; %.2f A; %.2f W; %.2f Wh; %.2f Hz; %.2f\n", 
      params->voltage, params->current, params->power, params->energy, params->frequency, params->pf );
    vTaskDelay(pdMS_TO_TICKS(200)); // Delay for 2 seconds
  }
}

// Task function to handle MQTT communication
void MQTTTask(void *pvParameters) {
  client.setServer(mqtt_server, 1883);
  PZEM_task_params *params = (PZEM_task_params*) pvParameters;
  while(1) 
  {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();

    char voltageStr[16];
    dtostrf(params->voltage, 1, 2, voltageStr);
    client.publish(voltage_topic, voltageStr);

    vTaskDelay(pdMS_TO_TICKS(10));

    char currentStr[16];
    dtostrf(params->current, 1, 2, currentStr);
    client.publish(current_topic, currentStr);

    vTaskDelay(pdMS_TO_TICKS(10));

    char powerStr[16];
    dtostrf(params->power, 1, 2, powerStr);
    client.publish(power_topic, powerStr);

    vTaskDelay(pdMS_TO_TICKS(10));

    char energyStr[16];
    dtostrf(params->energy, 1, 2, energyStr);
    client.publish(energy_topic, energyStr);

    vTaskDelay(pdMS_TO_TICKS(10));

    char frequencyStr[16];
    dtostrf(params->frequency, 1, 2, frequencyStr);
    client.publish(frequency_topic, frequencyStr);

    vTaskDelay(pdMS_TO_TICKS(10));

    char pfStr[16];
    dtostrf(params->pf, 1, 2, pfStr);
    client.publish(pf_topic, pfStr);

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void relayTask(void *pvParameters) {
  pinMode(RELAY_PIN, OUTPUT);
  while (true) {
    digitalWrite(RELAY_PIN, HIGH); // Turn relay ON
    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for 5 seconds
    digitalWrite(RELAY_PIN, LOW); // Turn relay OFF
    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for 5 seconds
  }

}

void checkFirmwareUpdate(void *pvParameters) {
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
      
      Serial.println("Target URL: " + firmwareUrl);

      t_httpUpdate_return ret = httpUpdate.update(client, firmwareUrl);

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
    vTaskDelay(pdMS_TO_TICKS(1000*180));
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();

  espSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  // Create the ReadPZEMTask pinned to core 1
  xTaskCreatePinnedToCore(
      ReadPZEMTask,          // Task function
      "ReadPZEMTask",       // Name of the task
      4096,                 // Stack size in words
      &PZEMParams,                 // Task input parameter
      1,                    // Priority of the task
      &ReadPZEMTaskHandle,  // Task handle
      1                     // Core where the task should run
  );
  
  //Create the MQTTTask pinned to core 1
  xTaskCreatePinnedToCore(
       MQTTTask,             // Task function
       "MQTTTask",          // Name of the task
       8192,                // Stack size in words
       &PZEMParams,                // Task input parameter
       1,                   // Priority of the task
       NULL,                // Task handle
       0                    // Core where the task should run
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
    10,
    NULL,
    0
  );

}

void loop() {

}