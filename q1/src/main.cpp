/////////--------IOT--------FIAP------------///////////
/////////--------SIMULAÇÃO GALPÃO MANGÁ------///////////

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
// #include <DHTesp.h>
#include <PubSubClient.h>


// Configurações de WiFi
const char *SSID = "Wokwi-GUEST";
const char *PASSWORD = "";

// Configurações de MQTT
const char *BROKER_MQTT = "broker.hivemq.com";
const int BROKER_PORT = 1883;
const char *ID_MQTT = "Catech_ESP32"; // Mantenha ou troque se precisar
const char *TOPIC_SUBSCRIBE_LED = "fe20/iot/led";
const char *TOPIC_PUBLISH_TEMP_HUMI = "catech_tempumi"; // Este é o tópico que o Node-RED vai ouvir

// Configurações de Hardware
// #define PIN_DHT 12
#define PIN_LED 15
#define PUBLISH_DELAY 2000


// Variáveis globais
WiFiClient espClient;
PubSubClient MQTT(espClient);
// DHTesp dht;
unsigned long publishUpdate = 0;
// TempAndHumidity sensorValues;
const int TAMANHO = 200;

// Variáveis para lógica de "só publicar se mudar"
float lastTemperature = 0.0; // Iniciando com 0 para forçar a primeira publicação
float lastHumidity = 0.0;
bool lastLedState = LOW;

// Protótipos de funções
// void updateSensorValues();
void initWiFi();
void initMQTT();
void callbackMQTT(char *topic, byte *payload, unsigned int length);
void reconnectMQTT();
void reconnectWiFi();
void checkWiFIAndMQTT();

// void updateSensorValues() {
//  sensorValues = dht.getTempAndHumidity();
// }

void initWiFi() {
  Serial.print("Conectando com a rede: ");
  Serial.println(SSID);
  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Conectado com sucesso: ");
  Serial.println(SSID);
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // Inicia o gerador de números aleatórios
  randomSeed(analogRead(0));
}

void initMQTT() {
  MQTT.setServer(BROKER_MQTT, BROKER_PORT);
  MQTT.setCallback(callbackMQTT);
}

// Esta função continua igual, para você poder "controlar" o LED
void callbackMQTT(char *topic, byte *payload, unsigned int length) {
  String msg = String((char*)payload).substring(0, length);

  JsonDocument json;
  DeserializationError error = deserializeJson(json, msg);

  if (error) {
    Serial.println("Falha na deserialização do JSON.");
    return;
  }

  if (json["led"].is<int>()) {
    int valor = json["led"];
    if (valor == 1) {
      digitalWrite(PIN_LED, HIGH);
      Serial.println("LED ligado via comando MQTT");
    } else if (valor == 0) {
      digitalWrite(PIN_LED, LOW);
      Serial.println("LED desligado via comando MQTT");
    }
  }
}

void reconnectMQTT() {
  while (!MQTT.connected()) {
    Serial.print("Tentando conectar com o Broker MQTT: ");
    Serial.println(BROKER_MQTT);

    if (MQTT.connect(ID_MQTT)) {
      Serial.println("Conectado ao broker MQTT!");
      MQTT.subscribe(TOPIC_SUBSCRIBE_LED);
    } else {
      Serial.println("Falha na conexão com MQTT. Tentando novamente em 2 segundos.");
      delay(2000);
    }
  }
}

void checkWiFIAndMQTT() {
  if (WiFi.status() != WL_CONNECTED) reconnectWiFi();
  if (!MQTT.connected()) reconnectMQTT();
}

void reconnectWiFi(void)
{
  if (WiFi.status() == WL_CONNECTED)
    return;

  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Wifi conectado com sucesso");
  Serial.print(SSID);
  Serial.println("IP: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // dht.setup(PIN_DHT, DHTesp::DHT22);

  initWiFi();
  initMQTT();
}

void loop() {
  checkWiFIAndMQTT();
  MQTT.loop();

  if ((millis() - publishUpdate) >= PUBLISH_DELAY) {
    publishUpdate = millis();

    // updateSensorValues(); // Substituído pela simulação abaixo

    // ####### INÍCIO DA SIMULAÇÃO #######

    // Simula Temperatura entre 18.00 e 26.00
    // random(min, max_exclusivo)
    float newTemperature = random(1800, 2601) / 100.0;

    // Simula Umidade entre 40.00 e 60.00
    float newHumidity = random(4000, 6001) / 100.0;

    bool currentLedState = digitalRead(PIN_LED);

    // ####### FIM DA SIMULAÇÃO #######


    // Lógica para só publicar se o valor mudar (ótima prática)
    bool changed =
      newTemperature != lastTemperature ||
      newHumidity != lastHumidity ||
      currentLedState != lastLedState;

    if (changed) {
      lastTemperature = newTemperature;
      lastHumidity = newHumidity;
      lastLedState = currentLedState;

      JsonDocument doc;
      doc["temperatura"] = lastTemperature;
      doc["umidade"] = lastHumidity;
      doc["status_led"] = lastLedState ? "on" : "off";

      char buffer[TAMANHO];
      serializeJson(doc, buffer);
      MQTT.publish(TOPIC_PUBLISH_TEMP_HUMI, buffer);

      Serial.print("Publicado (mudou): ");
      Serial.println(buffer);
    } else {
      Serial.println("Valores iguais, não publicando.");
    }
  }
}