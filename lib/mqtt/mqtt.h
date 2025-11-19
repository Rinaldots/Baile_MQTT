#include <WiFi.h>
#include <PubSubClient.h>

extern WiFiClient espClient;                           
extern PubSubClient client;                 
extern IPAddress mqtt_server;

class MqttTask {
    public:
    bool MQTT_Connected = false;                    
    void publish(const std::string& topic, const std::string& message);
    bool reconnect();
     
    // MQTT loop
    void handler();
    void callback(char* topic, byte* payload, unsigned int length);
    
    // Telemetria
    void updateTelemetry();
    
    // MQTT setup
    void setup();

    //Funções legado
    void girE(int tempoGiro);
    void girD(int tempoGiro);
    void para();
    void movF(int tempoMov);
    void movT(int tempoMov);
    void coreo();
    void rotD_F(int tempoMov);
    void rotE_F(int tempoMov);
    void rotD_T(int tempoMov);
    void rotE_T(int tempoMov);
    void ligaLeds();
    void acendeLeds(int led1, int led2, int led3, int led4, int led5, int led6, int led7, int led8);   
    void desligaLeds();
    private:
    bool ensureWifiConnected();
    void waitForWifi();
    };