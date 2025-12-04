#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <PZEM004Tv30.h>
#include <HardwareSerial.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <time.h>

// SSID and PASS
const char* ssid     = "b401_wifi";
const char* password = "b401juara1";

// MQTT Broker
const char* mqtt_server = "192.168.200.246";

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
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

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
    lagi:
        float voltage = pzem.voltage();
        float current = pzem.current();
        float power = pzem.power();
        float energy = pzem.energy();
        float frequency = pzem.frequency();
        float pf = pzem.pf();

        Serial.printf("%.2f V; %.2f A; %.2f W; %.2f Wh; %.2f Hz; %.2f\n", voltage, current, power, energy, frequency, pf );
        // Serial.printf("Current: %.2f A", current );
        // Serial.printf("Power: %.2f W", power );
        // Serial.printf("Energy: %.2f Wh", energy );
        // Serial.printf("Frequency: %.2f Hz", frequency );
        // Serial.printf("Power Factor: %.2f", pf);
        // Serial.println("-----------------------");

        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 2 seconds
        goto lagi;    
}

// Task function to handle MQTT communication
void MQTTTask(void *pvParameters) {
  setup_wifi();
  client.setServer(mqtt_server, 1883);

  for(;;) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();

    long now = millis();
    if (now - lastMsg > interval) {
      lastMsg = now;

      float voltage = pzem.voltage();
      float current = pzem.current();
      float power = pzem.power();
      float energy = pzem.energy();
      float frequency = pzem.frequency();
      float pf = pzem.pf();

      char voltageStr[8];
      dtostrf(voltage, 1, 2, voltageStr);
      client.publish(voltage_topic, voltageStr);

      char currentStr[8];
      dtostrf(current, 1, 2, currentStr);
      client.publish(current_topic, currentStr);

      char powerStr[8];
      dtostrf(power, 1, 2, powerStr);
      client.publish(power_topic, powerStr);

      char energyStr[8];
      dtostrf(energy, 1, 2, energyStr);
      client.publish(energy_topic, energyStr);

      char frequencyStr[8];
      dtostrf(frequency, 1, 2, frequencyStr);
      client.publish(frequency_topic, frequencyStr);

      char pfStr[8];
      dtostrf(pf, 1, 2, pfStr);
      client.publish(pf_topic, pfStr);
    }

    vTaskDelay(100 / portTICK_PERIOD_MS); // Short delay to prevent watchdog reset
  }
}


void setup() {
  Serial.begin(9600);

  espSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  // Create the ReadPZEMTask pinned to core 1
  xTaskCreatePinnedToCore(
      ReadPZEMTask,          // Task function
      "ReadPZEMTask",       // Name of the task
      4096,                 // Stack size in words
      NULL,                 // Task input parameter
      1,                    // Priority of the task
      &ReadPZEMTaskHandle,  // Task handle
      1                     // Core where the task should run
  );
  
  // Create the MQTTTask pinned to core 1
  // xTaskCreatePinnedToCore(
  //     MQTTTask,             // Task function
  //     "MQTTTask",          // Name of the task
  //     8192,                // Stack size in words
  //     NULL,                // Task input parameter
  //     1,                   // Priority of the task
  //     NULL,                // Task handle
  //     1                    // Core where the task should run
  // );

}

void loop() {

}