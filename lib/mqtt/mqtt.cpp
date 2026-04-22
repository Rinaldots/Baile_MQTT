#include "mqtt.h"
#include "diff_car.h"
#include <QuickPID.h>
#include <Preferences.h>
#include <Arduino.h>
#include <cstring>
#include <time.h> // Incluído para rotinas de tempo
#include "esp_wifi.h"
#define DEBUG_MQTT

extern QuickPID left_pid;

extern QuickPID right_pid;

WiFiClient espClient;
PubSubClient client(espClient);
IPAddress mqtt_server(10, 0, 0, 134);

namespace {
constexpr unsigned long WIFI_RETRY_INTERVAL_MS = WIFI_RETRY_DELAY_MS;
constexpr unsigned long WIFI_CONNECT_WAIT_STEP_MS = 500;
unsigned long lastWifiAttemptMs = 0;
bool wifiBeginIssued = false;
bool wifiWarnedMissingCredentials = false;
String pending_swarm_cmd = "";
}

void MqttTask::setup(String id) {
  robot_id = id;
  base_topic = "/" + robot_id;
  geral_topic = "/geral"; 
  time_t clocknow;
  waitForWifi();
  client.setServer(mqtt_server, 1883); 
  client.setBufferSize(512); // Tamanho normal do pacote sem payload do CSI
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
    String clientId = robot_id + "-";
    clientId += String(random(0xffff), HEX);

    // Tentativa de conexão:
    if(client.connect(clientId.c_str())) {
      MQTT_Connected = true;
      Serial.println("connected");
      client.subscribe((base_topic + "/cmd").c_str());
      client.subscribe((base_topic + "/parametros").c_str());
      
      
      // Tópicos do enxame:
      client.subscribe((geral_topic + "/cmd").c_str());
      client.subscribe((geral_topic + "/exec").c_str());
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

  if (String(topic) == geral_topic + "/cmd") {
      pending_swarm_cmd = mensagem;
      String ack_msg = "{\"id\":\"" + robot_id + "\", \"status\":\"ACK\"}";
      publish(std::string((geral_topic + "/ack").c_str()), ack_msg.c_str());
      Serial.println("[SWARM] Comando do enxame bufferizado. Enviando ACK para autorização.");
      return;
  }

  if (String(topic) == geral_topic + "/exec") {
      if (pending_swarm_cmd.length() > 0) {
          Serial.println("[SWARM] Autorização de execução (Commit) iniciada!");
          mensagem = pending_swarm_cmd; // Sobrescreve para que a lógica abaixo execute
          pending_swarm_cmd = ""; // Limpa do buffer
      } else {
          return;
      }
  }



  // Defensive checks before indexing
  if (mensagem.length() >= 3 && mensagem[0] == 'D' && mensagem[1] == 'N' && mensagem[2] == 'X') {
    if (mensagem.length() >= 6) {
      if (mensagem[4] == 'C' && mensagem[5] == 'G') {
        Serial.println("Iniciando coreografia!");
        coreo();
      } else if (mensagem[4] == 'M' && mensagem[5] == 'F') {
        Serial.println("Movendo para frente!");
        movF_Reto(100);
      } else if (mensagem[4] == 'M' && mensagem[5] == 'T') {
        movT_Reto(100);
      } else if (mensagem[4] == 'M' && mensagem[5] == 'D') {
        MovDistancia(100, 0.3f);
      } else if (mensagem[4] == 'M' && mensagem[5] == 'E') {
        MovDistancia(-100, 0.3f);
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
        
      } else if (mensagem[4] == 'S' && mensagem[5] == 'Y') {
        // Comando SYN (Ex: DNX=SYNC:1712345678) 
        // Em casos em que os robôs não tenham internet para os servidores NTP
        if (mensagem[6] == 'N' && mensagem[7] == 'C' && mensagem[8] == ':') {
          long int ext_time = mensagem.substring(9).toInt();
          struct timeval tv = { .tv_sec = ext_time, .tv_usec = 0 };
          settimeofday(&tv, NULL);
          Serial.printf("[SYNC] Tempo ajustado via MQTT para: %ld\n", ext_time);
        }
      } else if (mensagem[4] == 'G' && mensagem[5] == 'O') {
        int idx1 = mensagem.indexOf(':', 7);
        int idx2 = mensagem.indexOf(':', idx1 + 1);
        if (idx1 > 0 && idx2 > 0) {
           diffCar.target_x = mensagem.substring(7, idx1).toFloat();
           diffCar.target_y = mensagem.substring(idx1 + 1, idx2).toFloat();
           diffCar.target_theta = mensagem.substring(idx2 + 1).toFloat();
           diffCar.mode = 1; 
        }
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

void MqttTask::publishOdometry() {
    if (!client.connected()) return;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    unsigned long long timestamp_ms = (unsigned long long)(tv.tv_sec) * 1000 + (unsigned long long)(tv.tv_usec) / 1000;

    char msg[128];

    snprintf(msg, sizeof(msg),
        "{\"x\":%.3f,\"y\":%.3f,\"theta\":%.3f,\"t\":%llu}",
        diffCar.odom_real.x / 100.0f,
        diffCar.odom_real.y / 100.0f,
        diffCar.odom_real.theta,
        timestamp_ms
    );

    publish((base_topic + "/odom").c_str(), msg);
}

void MqttTask::publishVelocity() {
    if (!client.connected()) return;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    unsigned long long timestamp_ms = (unsigned long long)(tv.tv_sec) * 1000 + (unsigned long long)(tv.tv_usec) / 1000;

    char msg[128];

    snprintf(msg, sizeof(msg),
        "{\"left\":%.2f,\"right\":%.2f,\"target_l\":%.2f,\"target_r\":%.2f,\"t\":%llu}",
        diffCar.left_velocity_cms,
        diffCar.right_velocity_cms,
        diffCar.left_velocity_target,
        diffCar.right_velocity_target,
        timestamp_ms
    );

    publish((base_topic + "/velocity").c_str(), msg);
}

void MqttTask::publishIMU() {
    if (!client.connected()) return;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    unsigned long long timestamp_ms = (unsigned long long)(tv.tv_sec) * 1000 + (unsigned long long)(tv.tv_usec) / 1000;

    char msg[192];

    snprintf(msg, sizeof(msg),
        "{\"accel\":[%.3f,%.3f,%.3f],"
        "\"gyro\":[%.3f,%.3f,%.3f],"
        "\"yaw\":%.3f,\"t\":%llu}",
        diffCar.accel_d[0],
        diffCar.accel_d[1],
        diffCar.accel_d[2],
        diffCar.gyro_d[0],
        diffCar.gyro_d[1],
        diffCar.gyro_d[2],
        diffCar.ypr_d[0],
        timestamp_ms
    );

    publish((base_topic + "/imu").c_str(), msg);
}

void MqttTask::publishPID() {

    static unsigned long last = 0;
    if (millis() - last < 5000) return;
    last = millis();

    char msg[128];

    snprintf(msg, sizeof(msg),
        "{\"kp_l\":%.2f,\"ki_l\":%.2f,\"kd_l\":%.2f,"
        "\"kp_r\":%.2f,\"ki_r\":%.2f,\"kd_r\":%.2f}",
        left_pid.GetKp(), left_pid.GetKi(), left_pid.GetKd(),
        right_pid.GetKp(), right_pid.GetKi(), right_pid.GetKd()
    );

    publish((base_topic + "/pid").c_str(), msg);
}

void MqttTask::publishDiscovery() {

    static unsigned long last = 0;
    if (millis() - last < 1000) return;
    last = millis();

    char msg[64];

    snprintf(msg, sizeof(msg),
        "{\"id\":\"%s\"}",
        WiFi.macAddress().c_str()
    );

    publish((geral_topic + "/robots").c_str(), msg);
}

void MqttTask::updateTelemetry() {

    if (!client.connected()) return;

    publishOdometry();
    publishVelocity();
    publishIMU();
    publishPID();
    publishDiscovery();
}

