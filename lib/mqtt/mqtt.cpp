#include "mqtt.h"
#include "diff_car.h"
#include "network_config.h"
#include <Arduino.h>
#include <cstring>
#define DEBUG_MQTT

WiFiClient espClient;
PubSubClient client(espClient);
IPAddress mqtt_server(10, 0, 0, 134);

namespace {
constexpr unsigned long WIFI_RETRY_INTERVAL_MS = WIFI_RETRY_DELAY_MS;
constexpr unsigned long WIFI_CONNECT_WAIT_STEP_MS = 500;
unsigned long lastWifiAttemptMs = 0;
bool wifiBeginIssued = false;
bool wifiWarnedMissingCredentials = false;
}

void MqttTask::setup() {
  waitForWifi();
  client.setServer(mqtt_server, 1883); 
  client.setCallback([this](char* topic, byte* payload, unsigned int length) {
    this->callback(topic, payload, length);
  });
}

bool MqttTask::reconnect() {
  if (!ensureWifiConnected()) {
    return false;
  }
  // Loop repetido até o estabelecimento da reconexão ->
  if (!client.connected()) {
    MQTT_Connected = false;
    Serial.print("Attempting MQTT connection...");

    // Criação de um "Client ID" randômico:
    String clientId = "dan1";
    clientId += String(random(0xffff), HEX);

    // Tentativa de conexão:
    if(client.connect(clientId.c_str())) {
      MQTT_Connected = true;
      Serial.println("connected");
      client.subscribe("cmd");
    }else{
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
    }
  }
  return client.connected();
}

void MqttTask::callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    String messageTemp;
    
    for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
      messageTemp += (char)payload[i];
    }
    Serial.println();

  // Build a safe String from payload (messageTemp already contains it)
  messageTemp.trim(); // remove trailing newline/carriage returns
  String mensagem = messageTemp;

  // Defensive checks before indexing
  if (mensagem.length() >= 3 && mensagem[0] == 'D' && mensagem[1] == 'N' && mensagem[2] == 'X') {
    if (mensagem.length() >= 6) {
      if (mensagem[4] == 'C' && mensagem[5] == 'G') {
        Serial.println("Iniciando coreografia!");
        coreo();
      } else if (mensagem[4] == 'M' && mensagem[5] == 'F') {
        Serial.println("Movendo para frente!");
        movF(1);
      } else if (mensagem[4] == 'M' && mensagem[5] == 'T') {
        movT(1);
      } else if (mensagem[4] == 'G' && mensagem[5] == 'D') {
        girD(1);
      } else if (mensagem[4] == 'G' && mensagem[5] == 'E') {
        girE(1);
      } else if (mensagem[4] == 'P' && mensagem[5] == 'A') {
        para();
      } else if (mensagem[4] == 'N' && mensagem[5] == 'V') {
        diffCar.mode = 1;
        double dist = mensagem.substring(7).toDouble();
        if (mensagem[6] == '-'){
          dist = -dist;
        }
        diffCar.navegar_reto(dist);
      } else {
        Serial.print("Unknown DNX command: ");
        Serial.println(mensagem.substring(4));
      }
    } else {
      Serial.println("DNX message too short");
    }
  }
}

void MqttTask::handler() {
  if (!ensureWifiConnected()) {
    return;
  }
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}

bool MqttTask::ensureWifiConnected() {
  if (strlen(WIFI_SSID) == 0 && !wifiWarnedMissingCredentials) {
    Serial.println("[WiFi] WIFI_SSID not configured. Define WIFI_SSID/WIFI_PASSWORD in network_config.h or via build_flags.");
    wifiWarnedMissingCredentials = true;
    return false;
  }

  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  const unsigned long now = millis();
  if (!wifiBeginIssued || now - lastWifiAttemptMs >= WIFI_RETRY_INTERVAL_MS) {
    Serial.print("[WiFi] Connecting to SSID: ");
    Serial.println(WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    wifiBeginIssued = true;
    lastWifiAttemptMs = now;
  }
  return false;
}

void MqttTask::waitForWifi() {
  const unsigned long start = millis();
  while (millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    if (ensureWifiConnected()) {
      Serial.print("[WiFi] Connected, IP: ");
      Serial.println(WiFi.localIP());
      return;
    }
    delay(WIFI_CONNECT_WAIT_STEP_MS);
  }
  Serial.println("[WiFi] Connection timeout. MQTT will retry in background.");
}

void MqttTask::publish(const std::string& topic, const std::string& message) {
  if (client.connected()) {
    client.publish(topic.c_str(), message.c_str());
  }
}

void MqttTask::updateTelemetry() {
  if (!client.connected()) {
    return;
  }
  // Exemplo de publicação de telemetria
  String telemetry = "{";
  telemetry += "\"left_velocity_ms\":" + String(diffCar.left_velocity_ms) + ",";
  telemetry += "\"right_velocity_ms\":" + String(diffCar.right_velocity_ms) + ",";
  telemetry += "}";
  publish("telemetry", telemetry.c_str());
}

