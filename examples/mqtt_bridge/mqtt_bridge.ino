#include <ArduinoJson.h>
#include <Ethernet.h>
#include <M5Stack.h>
#include <PubSubClient.h>
#include <SPI.h>

#include "cybergear_m5/cybergear_can_interface_esp32.hh"
#include "cybergear_m5/cybergear_driver.hh"

// Update these with values suitable for your network.
#define DEVICE_NAME "client"
#define QOS 0
#define SCK 18
#define MISO 19
#define MOSI 23
#define CS 26

byte mac[] = {0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED};
IPAddress ip(192, 168, 50, 200);
IPAddress server(192, 168, 50, 76);

// setup master can id and motor can id (default cybergear can id is 0x7F)
uint8_t MASTER_CAN_ID = 0x00;
uint8_t MOT_CAN_ID = 0x7F;
unsigned long send_cnt = 0;
unsigned long send_start_time = 0;
unsigned long recv_cnt = 0;
unsigned long recv_start_time = 0;

// init cybergeardriver
CybergearDriver driver = CybergearDriver(MASTER_CAN_ID, MOT_CAN_ID);
CybergearCanInterfaceEsp32 interface;
MotorStatus motor_status;

// Callback function header
void callback(char * topic, byte * payload, unsigned int length);
bool connect_mqtt_server();

EthernetClient eth_client;
PubSubClient mqtt_client(eth_client);

void callback(char * topic, byte * payload, unsigned int length)
{
  recv_cnt++;
  if (recv_cnt > 1000) {
    unsigned long diff = millis() - recv_start_time;
    Serial.printf("recv = %u\n", diff);
    recv_cnt = 0;
    recv_start_time = millis();
  }

  Serial.printf("name(%u) : %s - ", length, topic);
  for (uint16_t idx = 0; idx < length; ++idx) Serial.printf("%c(%x) ", payload[idx], payload[idx]);
  Serial.println("");
}

void setup()
{
  M5.begin(true, false, true);

  // Setup Ethernet and mqtt client
  Ethernet.init(CS);
  Ethernet.begin(mac, ip);
  while (!connect_mqtt_server()) {
    Serial.printf("Waiting for host.\n");
    delay(1000);
  }
  Serial.printf("Connect to mqtt server.\n");

  // setup cybergear
  interface.init(5, 15);
  driver.init(&interface);
  driver.init_motor(MODE_CURRENT);
  Serial.printf("Init cybergear driver.\n");

  send_start_time = millis();
  recv_start_time = millis();
}

bool connect_mqtt_server()
{
  if (mqtt_client.connected()) {
    return true;
  }

  mqtt_client.setServer(server, 1883);
  mqtt_client.setCallback(callback);
  if (!mqtt_client.connect(DEVICE_NAME)) {
    Serial.printf("Mqtt connection failed as [%s].\n", DEVICE_NAME);
    return false;
  }

  mqtt_client.subscribe("joint_commands", QOS);
  return true;
}

void loop()
{
  M5.update();
  driver.set_current_ref(0.0f);
  if (driver.process_packet()) {
    motor_status = driver.get_motor_status();
  }

  if (connect_mqtt_server()) {
    JsonDocument doc;
    doc["name"][0] = "test";
    doc["position"][0] = motor_status.position;
    doc["velocity"][0] = motor_status.velocity;
    doc["effort"][0] = motor_status.effort;
    String output;
    serializeJson(doc, output);
    mqtt_client.publish("joint_states", output.c_str());
  }
  mqtt_client.loop();

  send_cnt++;
  if (send_cnt > 1000) {
    unsigned long diff = millis() - send_start_time;
    Serial.printf("send %u\n", diff);
    send_cnt = 0;
    send_start_time = millis();
  }
}
