#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <PZEM004Tv30.h>
#include <HardwareSerial.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <time.h>
#include "secret.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// SSID and PASS
const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASSWORD;

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
    vTaskDelay(pdMS_TO_TICKS(2000)); // Delay for 2 seconds
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

    vTaskDelay(pdMS_TO_TICKS(2000));
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
  /*xTaskCreatePinnedToCore(
       MQTTTask,             // Task function
       "MQTTTask",          // Name of the task
       8192,                // Stack size in words
       &PZEMParams,                // Task input parameter
       1,                   // Priority of the task
       NULL,                // Task handle
       1                    // Core where the task should run
   );*/

}

void loop() {

}