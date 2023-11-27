#include <Arduino.h>
#include <math.h>
#include <mcp_can.h>
#include <M5Stack.h>
#include <numeric>
#include <vector>
#include "ros2_logo.hh"
#include "cybergear_bridge.hh"
#include "cybergear_controller.hh"


/**
 * @brief Init can interface
 */
void init_can();

// init MCP_CAN object
#define CAN0_INT 15  // Set INT to pin 2
MCP_CAN CAN0(12);    // Set CS to pin 10

// setup master can id and motor can id (default cybergear can id is 0x7F)
const uint8_t MASTER_CAN_ID = 0x00;
const uint8_t MOT_JOINT_1 = 0x7F;
const uint8_t MOT_JOINT_2 = 0x7E;
const uint8_t MOT_JOINT_3 = 0x7D;
std::vector<uint8_t> motor_ids = {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3};
std::vector<float> init_pose = {0.0f, -M_PI/4.0, 0.0f};
std::vector<int8_t> calib_dir = {CW, CW, CW};
std::vector<CybergearSoftwareConfig> sw_configs = {
   // id, direction, limit_speed, limit_current, limit_torque, upper_pos_limit, lower_pos_limit, pos_offset
  CybergearSoftwareConfig(MOT_JOINT_1, CCW, 1.0f, 1.0f, 0.5f, (float)65.0/180.0*M_PI, (float)-65.0/180.0*M_PI, CCW, (float)-65.0/180.0*M_PI),
  CybergearSoftwareConfig(MOT_JOINT_2, CW,  1.0f, 1.0f, 0.5f, (float)M_PI/2.0f, (float)-M_PI/4.0f, CCW, (float)-M_PI/2.0f),
  CybergearSoftwareConfig(MOT_JOINT_3, CCW, 1.0f, 1.0f, 0.5f, (float)M_PI/4.0f, (float)-M_PI, CW, (float)M_PI/4.0f)
};

// state
#define INITIAL_STATE           0
#define ORIGIN_DETECT_MODE      1
#define MOTION_CONTROL_MODE     2
#define POSITION_CONTROL_MODE   3
#define SPEED_CONTROL_MODE      4
#define CURRENT_CONTROL_MODE    5
#define ORIGIN_DETECT_TORQUE    0.3f
#define ORIGIN_DETECT_SPEED     1.0f
#define INITIAL_POSE_MOVE_SPEED 1.8f
std::vector<uint8_t> calib_motor_seq = {MOT_JOINT_3, MOT_JOINT_2, MOT_JOINT_1};
uint8_t current_mode = INITIAL_STATE;

// init cybergeardriver
CybergearController controller = CybergearController(MASTER_CAN_ID);
CybergearBridge bridge = CybergearBridge(&controller, &Serial);

// init sprite for display
TFT_eSprite sprite = TFT_eSprite(&sprite);
const float init_speed = 1.0f;        //!< slow speed
const float move_speed = 15.0f;       //!< quick speed

void setup()
{
  M5.begin(true,true,false);
  Serial.begin(2000000);
  Serial.flush();

  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.drawBitmap((M5.Lcd.width() - imgWidth)/2, (M5.Lcd.height() - imgHeight)/2, imgWidth, imgHeight, (uint16_t *)img);

  // init cybergear driver
  init_can();

  controller.init(motor_ids, sw_configs, MODE_CURRENT, &CAN0);

  for (uint8_t idx = 0; idx < motor_ids.size(); ++idx) {
    controller.set_speed_limit(motor_ids[idx], init_speed);
  }

  delay(1000);
  M5.Lcd.fillScreen(BLACK);

  // init sprite
  sprite.setColorDepth(8);
  sprite.setTextSize(2);
  sprite.createSprite(M5.Lcd.width(), M5.Lcd.height());
}

void loop()
{
  // update m5 satatus
  M5.update();

  if (M5.BtnA.wasPressed()) {
    calibrate_positoin_zero_offset();

  } else if (M5.BtnB.wasPressed()) {
    demo_mode();

  } else if (M5.BtnC.wasPressed()) {
    current_mode = INITIAL_STATE;
    controller.reset_motors();
    for (uint8_t idx = 0; idx < motor_ids.size(); ++idx) {
      controller.set_mech_position_to_zero(motor_ids[idx]);
    }
  }

  if (current_mode == POSITION_CONTROL_MODE) {
    bridge.process_request_command();
    bridge.process_motor_response();

  } else if (current_mode == INITIAL_STATE) {
    bridge.process_motor_response();
    controller.send_current_command(motor_ids, {0.0, 0.0, 0.0});
  }

  if ( controller.process_can_packet() ) {
    std::vector<MotorStatus> status_list;
    if (controller.get_motor_status(status_list)) {
      uint16_t bg_color = (current_mode == INITIAL_STATE) ? BLACK : BLUE;
      uint16_t txt_color = TFT_WHITE;
      sprite.fillScreen(bg_color);
      sprite.setCursor(0, 0);
      sprite.setTextColor(txt_color, bg_color);
      sprite.setTextSize(2);
      for (uint8_t idx = 0; idx < status_list.size(); ++idx) {
        sprite.printf("motor id  0x%x \n", status_list[idx].motor_id);
        sprite.printf("  position : %.03f deg\n", status_list[idx].position * 180.0 / M_PI);
        sprite.printf("  velocity : %.03f \n", status_list[idx].velocity);
        sprite.printf("  effort   : %.03f \n\n", status_list[idx].effort);
      }
      sprite.pushSprite(0, 0);
    }
  }
}

void init_can()
{
  CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);
  CAN0.setMode(MCP_NORMAL);  // Set operation mode to normal so the MCP2515 sends acks to received data.
  pinMode(CAN0_INT, INPUT);  // Configuring pin for /INT input
}

void calibrate_positoin_zero_offset()
{
  // reset motor
  controller.reset_motors();
  controller.set_run_mode(MODE_SPEED);
  controller.enable_motors();

  // stop motors
  for (uint8_t idx = 0; idx < motor_ids.size(); ++idx) {
    controller.send_speed_command(motor_ids[idx], 0.0f);
    controller.process_can_packet();
  }

  for (uint8_t idx = 0; idx < calib_motor_seq.size(); ++idx) {
    std::vector<float> efforts = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // get motor config
    CybergearSoftwareConfig config;
    if (!controller.get_software_config(calib_motor_seq[idx], config)) continue;

    while (std::abs(std::accumulate(efforts.begin(), efforts.end(), 0.0) / efforts.size()) < 0.6f || efforts.size() < 40) {

      // set motor speed
      controller.send_speed_command(calib_motor_seq[idx], config.calib_direction * ORIGIN_DETECT_SPEED);
      if (controller.process_can_packet()) {
        MotorStatus mot;
        controller.get_motor_status(calib_motor_seq[idx], mot);
        efforts.push_back(mot.effort);
      }

      // resize status
      while (efforts.size() > 50) efforts.erase(efforts.begin());
    }

    controller.set_mech_position_to_zero(calib_motor_seq[idx]);
    controller.send_speed_command(calib_motor_seq[idx], 0.0f);
  }

  controller.reset_motors();

  // setup motor
  controller.set_run_mode(MODE_POSITION);
  for (uint8_t idx = 0; idx < motor_ids.size(); ++idx) {
    controller.set_speed_limit(motor_ids[idx], INITIAL_POSE_MOVE_SPEED);
  }
  controller.enable_motors();

  // set up initial position
  std::vector<float> pos_diffs = {1000.0f, 1000.0f, 1000.0f};
  while (std::abs(std::accumulate(pos_diffs.begin(), pos_diffs.end(), 0.0) / pos_diffs.size()) > 1.0f / 180.0f * M_PI) {
    for (uint8_t idx = 0; idx < motor_ids.size(); ++idx) {
      controller.send_position_command(motor_ids[idx], init_pose[idx]);
      if (controller.process_can_packet()) {
        MotorStatus mot;
        controller.get_motor_status(motor_ids[idx], mot);
        pos_diffs[idx] = std::abs(mot.position - init_pose[idx]);
      }
    }
  }

  controller.reset_motors();
}


struct Motion
{
  float duration;
  std::vector<uint8_t> ids;
  std::vector<CybergearMotionCommand> cmds;
};


void demo_mode()
{
  controller.reset_motors();
  for (uint8_t idx = 0; idx < motor_ids.size(); ++idx) {
    controller.set_speed_limit(motor_ids[idx], move_speed);
  }
  controller.set_run_mode(MODE_POSITION);
  controller.enable_motors();

  std::vector<Motion> motion_seq = {
    {0.5f, {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3}, {
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
    }},
    {0.5f, {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3}, {
      {25.0/180.0 * M_PI, 1.0f, 0.0f, 1.0f, 1.0f},
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
    }},
    {0.5f, {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3}, {
      {-25.0/180.0 * M_PI, -1.0f, 0.0f, 1.0f, 1.0f},
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
    }},
    {0.5f, {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3}, {
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
    }},
    {0.5f, {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3}, {
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
      {40.0/180.0 * M_PI, 0.0f, 0.0f, 1.0f, 1.0f},
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
    }},
    {0.5f, {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3}, {
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
      {-40.0/180.0 * M_PI, 0.0f, 0.0f, 1.0f, 1.0f},
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
    }},
    {0.5f, {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3}, {
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
      {-40.0/180.0 * M_PI, 0.0f, 0.0f, 1.0f, 1.0f},
      {40.0/180.0 * M_PI, 0.0f, 0.0f, 1.0f, 1.0f},
    }},
    {1.0f, {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3}, {
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
      {-40.0/180.0 * M_PI, 0.0f, 0.0f, 1.0f, 1.0f},
      {-90.0/180.0 * M_PI, 0.0f, 0.0f, 1.0f, 1.0f},
    }},
    {0.5f, {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3}, {
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
    }},
    {0.5f, {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3}, {
      {25.0/180.0 * M_PI, 1.0f, 0.0f, 1.0f, 1.0f},
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
    }},
    {0.5f, {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3}, {
      {-25.0/180.0 * M_PI, -1.0f, 0.0f, 1.0f, 1.0f},
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
    }},
    {0.5f, {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3}, {
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
    }},
    {0.5f, {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3}, {
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
      {40.0/180.0 * M_PI, 0.0f, 0.0f, 1.0f, 1.0f},
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
    }},
    {0.5f, {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3}, {
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
      {-40.0/180.0 * M_PI, 0.0f, 0.0f, 1.0f, 1.0f},
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
    }},
    {0.5f, {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3}, {
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
      {-40.0/180.0 * M_PI, 0.0f, 0.0f, 1.0f, 1.0f},
      {40.0/180.0 * M_PI, 0.0f, 0.0f, 1.0f, 1.0f},
    }},
    {1.0f, {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3}, {
      {0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
      {-40.0/180.0 * M_PI, 0.0f, 0.0f, 1.0f, 1.0f},
      {-90.0/180.0 * M_PI, 0.0f, 0.0f, 1.0f, 1.0f},
    }}
  };

  for (uint8_t motion_idx = 0; motion_idx < motion_seq.size(); ++motion_idx) {
    unsigned long duration = static_cast<unsigned long>(motion_seq[motion_idx].duration * 1000.0f);
    unsigned long end_time = static_cast<unsigned long>(millis() + duration);
    while (millis() < end_time) {
      for (uint8_t mot_idx = 0; mot_idx < motion_seq[motion_idx].ids.size(); ++mot_idx) {
        // controller.send_motion_command(motion_seq[motion_idx].ids[mot_idx], motion_seq[motion_idx].cmds[mot_idx]);
        controller.send_position_command(motion_seq[motion_idx].ids[mot_idx], motion_seq[motion_idx].cmds[mot_idx].position);
        controller.process_can_packet();
      }
      delay(1);
    }
  }

  controller.reset_motors();
  for (uint8_t idx = 0; idx < motor_ids.size(); ++idx) {
    controller.set_speed_limit(motor_ids[idx], init_speed);
  }
}
