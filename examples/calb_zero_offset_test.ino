#include <Arduino.h>
#include <cstdio>
#include <math.h>
#include <mcp_can.h>
#include <M5Stack.h>
#include <ratio>
#include <numeric>
#include "cybergear_controller.hh"

/**
 * @brief Init can interface
 */
void init_can();

// init MCP_CAN object
#define CAN0_INT 15  // Set INT to pin 2
MCP_CAN CAN0(12);    // Set CS to pin 10

// setup master can id and motor can id (default cybergear can id is 0x7F)
uint8_t MASTER_CAN_ID = 0x00;
uint8_t MOT_CAN_ID = 0x7F;
const float MAX_CURRENT = 1.0;
const float LEADER_FOLLOWER_GAIN = 30.0;

// init sprite for display
TFT_eSprite sprite = TFT_eSprite(&sprite);

std::vector<uint8_t> motor_ids = {MOT_CAN_ID};
float speed = 0.8f;
float dir = 1.0f;
std::vector<float> efforts = {};

// init cybergeardriver
CybergearController controller = CybergearController(MASTER_CAN_ID);

void setup()
{
  M5.begin();

  // init cybergear driver
  init_can();

  // init sprite
  sprite.setColorDepth(8);
  sprite.setTextSize(2);
  sprite.createSprite(M5.Lcd.width(), M5.Lcd.height());

  // init position offset
  sprite.print("Init motors ... ");
  controller.init(motor_ids, MODE_POSITION, &CAN0);
  controller.enable_motors();
  sprite.println("done");

  sprite.print("move to motor origin ... ");
  controller.send_position_command(motor_ids, {0.0f});
  delay(1000);
  sprite.println("done");

  // start bilateral mode
  sprite.print("starting leader follower demo ... ");
  controller.init(motor_ids, MODE_SPEED, &CAN0);
  controller.enable_motors();
  M5.Lcd.println("done");
}

void loop()
{
  // update m5 satatus
  M5.update();

  controller.send_speed_command(MOT_CAN_ID, dir * speed);

  // update and get motor data
  MotorStatus status;
  if ( controller.process_can_packet() ) {
    controller.get_motor_status(MOT_CAN_ID, status);
  }

  efforts.push_back(status.effort);
  if (efforts.size() > 5) {
    while (efforts.size() > 5) efforts.erase(efforts.begin());
    float ave = std::accumulate(efforts.begin(), efforts.end(), 0.0) / efforts.size();
    if (std::abs(ave) > 0.2f) {
      dir *= -1.0f;
      efforts.clear();
      controller.set_mech_position_to_zero(MOT_CAN_ID);
    }
  }
}

void init_can()
{
  CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);
  CAN0.setMode(MCP_NORMAL);  // Set operation mode to normal so the MCP2515 sends acks to received data.
  pinMode(CAN0_INT, INPUT);  // Configuring pin for /INT input
}
