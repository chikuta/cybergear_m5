#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <math.h>
#include <mcp_can.h>
#include <M5Stack.h>
#include <numeric>
#include <stdexcept>
#include <sys/types.h>
#include <vector>
#include "ros2_logo.hh"
#include "cybergear_bridge.hh"
#include "cybergear_controller.hh"


struct Motion
{
  float duration;
  std::vector<uint8_t> ids;
  std::vector<CybergearMotionCommand> cmds;
};

/**
 * @brief Init can interface
 */
void init_can();
void calc_foward_kinematics(float j1, float f2, float j3);
void calc_inverse_kinematics(float x, float y, float z);
void execute_motion(const std::vector<Motion>& motion_seq);
void display_motor_state();

// for task
unsigned long int_cnt = 0;
QueueHandle_t msg_queue;
TaskHandle_t process_can_task_handle;
TaskHandle_t loop_task_handle;

// init MCP_CAN object
#define CAN0_INT 15  // Set INT to pin 2
MCP_CAN CAN0(12);    // Set CS to pin 10

// setup master can id and motor can id (default cybergear can id is 0x7F)
const uint8_t MASTER_CAN_ID = 0x00;
const uint8_t MOT_JOINT_1 = 0x7F;
const uint8_t MOT_JOINT_2 = 0x7E;
const uint8_t MOT_JOINT_3 = 0x7D;
std::vector<uint8_t> motor_ids = {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3};
std::vector<float> init_pose = {0.0f, M_PI/4, 0.0f};
std::vector<CybergearSoftwareConfig> sw_configs = {
   // id, direction, limit_speed, limit_current, limit_torque, upper_pos_limit, lower_pos_limit, pos_offset
  CybergearSoftwareConfig(MOT_JOINT_1, CCW, 1.0f, 1.0f, 0.5f, (float)65.0/180.0*M_PI, (float)-65.0/180.0*M_PI, CCW, (float)-65.0/180.0*M_PI),
  CybergearSoftwareConfig(MOT_JOINT_2, CCW, 1.0f, 1.0f, 0.5f, (float)M_PI/2.0f, (float)-M_PI/4.0f, CW, (float)M_PI/2),
  CybergearSoftwareConfig(MOT_JOINT_3, CW,  1.0f, 1.0f, 0.5f, (float)M_PI, (float)-M_PI/4.0f, CCW, (float)-30.0/180.0*M_PI)
};

// state
#define ORIGIN_DETECT_TORQUE    0.3f
#define ORIGIN_DETECT_SPEED     1.5f
#define INITIAL_POSE_MOVE_SPEED 1.8f
std::vector<uint8_t> calib_motor_seq = {MOT_JOINT_3, MOT_JOINT_2, MOT_JOINT_1};

enum struct State
{
  INIT,
  STOP,
  CALIBRATE_ORIGIN,
  CHANGE_TO_LOW_GAIN,
  LOW_GAIN,
  CHANGE_TO_HIGH_GAIN,
  HIGH_GAIN,
  CHANGE_TO_EXECUTE_MOTION,
  EXECUTE_MOTION,
  CHANGE_TO_DISPLAY_MOTOR_STATE,
  DISPLAY_MOTOR_STATE,
  ERROR,
};
State mode = State::INIT;

// init cybergeardriver
CybergearController controller = CybergearController(MASTER_CAN_ID);
CybergearBridge bridge = CybergearBridge(&controller, &Serial);

// init sprite for display
TFT_eSprite sprite = TFT_eSprite(&sprite);
const float init_speed = 2.0f;        //!< slow speed
const float move_speed = 10.0f;       //!< quick speed

const float LOW_GAIN_CURRENT_LIMIT = 2.0f;
const float LOW_GAIN_TORQUE_LIMIT = 2.0f;
const float LOW_GAIN_POSITION_KP = 30.0f;
const float LOW_GAIN_VELOCITY_KP = 2.0f;
const float LOW_GAIN_VELOCITY_KI = 0.021f;
const float LOW_GAIN_CURRENT_KP = 0.125f;
const float LOW_GAIN_CURRENT_KI = 0.0158f;
const float LOW_GAIN_CURRENT_GAIN = 0.1f;

const float HIGH_GAIN_CURRENT_LIMIT = 12.0f;
const float HIGH_GAIN_TORQUE_LIMIT = 10.0f;
const float HIGH_GAIN_POSITION_KP = 100.0f;
const float HIGH_GAIN_VELOCITY_KP = 8.0f;
const float HIGH_GAIN_VELOCITY_KI = 0.1f;
const float HIGH_GAIN_CURRENT_KP = 0.125f;
const float HIGH_GAIN_CURRENT_KI = 0.0158f;
const float HIGH_GAIN_CURRENT_GAIN = 0.1f;

void calc_foward_kinematics(float j1, float j2, float j3, float& x, float& y, float& z)
{
  float l1 = 0.10;
  float l2 = 0.15;
  float l3 = 0.15;
  float s1 = sin(j1);
  float c1 = cos(j1);
  float s2 = sin(j2);
  float c2 = cos(j2);
  float s23 = sin(j2 + j3);
  float c23 = cos(j2 + j3);
  x = c1 * (l2 * c2 + l3 * c23);
  y = s1 * (l2 * c2 + l3 * c23);
  z = l2 * s2 + l3 * s23;
}

void calc_inverse_kinematics(float x, float y, float z, float& j1, float& j2, float& j3)
{
  float l1 = 0.10;
  float l2 = 0.15;
  float l3 = 0.15;
  j1 = std::atan2(y, x);
  j3 = -std::acos((x*x + y*y + z*z - l2*l2 - l3*l3) / (2.0*l2*l3));
  float s3 = std::sin(j3);
  float c3 = std::cos(j3);
  j2 = std::atan2(-l3 * s3 * std::sqrt(x*x + y*y) + (l2 + l3 * c3) * z, (l2 + l3*c3) * std::sqrt(x*x + y*y) + l3*s3*z);
  // j2 = std::atan2(l3 * s3 * std::sqrt(x*x + y*y) + (l2 + l3 * c3) * z, -(l2 + l3*c3) * std::sqrt(x*x + y*y) + l3*s3*z);
  j2 -= M_PI/4.0f;
  j3 += M_PI/2.0f;
}

void setup()
{
  M5.begin(true,true,false);
  Serial.begin(2000000);
  // Serial.begin(115200);
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

void change_state()
{
  // state change event
  if (M5.BtnA.wasPressed()) {
    if (mode == State::STOP) {
      mode = State::CHANGE_TO_DISPLAY_MOTOR_STATE;
    }
    else if (mode == State::DISPLAY_MOTOR_STATE) {
      mode = State::INIT;
    }
    else if (mode == State::LOW_GAIN) {
      mode = State::INIT;
    }
    else if (mode == State::HIGH_GAIN) {
      mode = State::INIT;
    }
    else if (mode == State::EXECUTE_MOTION) {
      mode = State::INIT;
    }
  }
  else if (M5.BtnB.wasPressed()) {
    if (mode == State::STOP) {
      mode = State::CHANGE_TO_LOW_GAIN;
    }
    else if (mode == State::HIGH_GAIN) {
      mode = State::CHANGE_TO_LOW_GAIN;
    }
  }
  else if (M5.BtnC.wasPressed()) {
    if (mode == State::STOP) {
      mode = State::CALIBRATE_ORIGIN;
    }
    else if (mode == State::LOW_GAIN) {
      mode = State::CHANGE_TO_HIGH_GAIN;

    } else if (mode == State::HIGH_GAIN) {
      mode = State::CHANGE_TO_EXECUTE_MOTION;
    }
  }

  // execute mode
  if (mode == State::INIT) {
    // set mode to current
    M5.Lcd.fillScreen(BLUE);
    M5.Lcd.setTextColor(WHITE, BLUE);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextSize(4);
    M5.Lcd.setTextDatum(7);
    M5.Lcd.drawString("Stop Mode", M5.Lcd.width() / 2, M5.Lcd.height() / 2, 1);

    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextDatum(7);
    M5.Lcd.drawString("Info", M5.Lcd.width() / 2 - M5.Lcd.width() / 3, M5.Lcd.height() - 10, 1);
    M5.Lcd.drawString("Low", M5.Lcd.width() / 2, M5.Lcd.height() - 10, 1);
    // M5.Lcd.drawString("High", M5.Lcd.width() / 2 + M5.Lcd.width() / 3, M5.Lcd.height() - 10, 1);
    M5.Lcd.drawString("Calib", M5.Lcd.width() / 2 + M5.Lcd.width() / 3, M5.Lcd.height() - 10, 1);

    controller.reset_motors();
    for (uint8_t idx = 0; idx < motor_ids.size(); ++idx) {
      controller.set_speed_limit(motor_ids[idx], init_speed);
      controller.set_torque_limit(motor_ids[idx], LOW_GAIN_TORQUE_LIMIT);
      controller.set_current_limit(motor_ids[idx], LOW_GAIN_CURRENT_LIMIT);
      controller.set_position_control_gain(motor_ids[idx], LOW_GAIN_POSITION_KP);
      controller.set_velocity_control_gain(motor_ids[idx], LOW_GAIN_VELOCITY_KP, LOW_GAIN_VELOCITY_KI);
      controller.set_current_control_param(motor_ids[idx], LOW_GAIN_CURRENT_KP, LOW_GAIN_CURRENT_KI, LOW_GAIN_CURRENT_GAIN);
    }
    mode = State::STOP;

  } else if (mode == State::STOP) {
    // set zero torque
    // DO NOTHING

  } else if (mode == State::DISPLAY_MOTOR_STATE) {
    display_motor_state();

  } else if (mode == State::CALIBRATE_ORIGIN) {
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextSize(4);
    M5.Lcd.setTextDatum(7);
    M5.Lcd.drawString("Calib Mode", M5.Lcd.width() / 2, M5.Lcd.height() / 2, 1);

    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextDatum(7);
    // M5.Lcd.drawString("Stop", M5.Lcd.width() / 2 - M5.Lcd.width() / 3, M5.Lcd.height() - 10, 1);
    // M5.Lcd.drawString("Low", M5.Lcd.width() / 2, M5.Lcd.height() - 10, 1);
    // M5.Lcd.drawString("High", M5.Lcd.width() / 2 + M5.Lcd.width() / 3, M5.Lcd.height() - 10, 1);
    // M5.Lcd.drawString("Calib", M5.Lcd.width() / 2 + M5.Lcd.width() / 3, M5.Lcd.height() - 10, 1);

    calibrate_positoin_zero_offset();
    mode = State::INIT;

  } else if (mode == State::CHANGE_TO_DISPLAY_MOTOR_STATE) {
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextSize(4);
    M5.Lcd.setTextDatum(7);
    mode = State::DISPLAY_MOTOR_STATE;

  } else if (mode == State::CHANGE_TO_LOW_GAIN) {
    M5.Lcd.fillScreen(GREEN);
    M5.Lcd.setTextColor(WHITE, GREEN);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextSize(4);
    M5.Lcd.setTextDatum(7);
    M5.Lcd.drawString("LowGain Mode", M5.Lcd.width() / 2, M5.Lcd.height() / 2, 1);

    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextDatum(7);
    M5.Lcd.drawString("Stop", M5.Lcd.width() / 2 - M5.Lcd.width() / 3, M5.Lcd.height() - 10, 1);
    // M5.Lcd.drawString("Low", M5.Lcd.width() / 2, M5.Lcd.height() - 10, 1);
    M5.Lcd.drawString("High", M5.Lcd.width() / 2 + M5.Lcd.width() / 3, M5.Lcd.height() - 10, 1);
    // M5.Lcd.drawString("Calib", M5.Lcd.width() / 2 + M5.Lcd.width() / 3, M5.Lcd.height() - 10, 1);

    controller.reset_motors();
    controller.set_run_mode(MODE_POSITION);
    for (uint8_t idx = 0; idx < motor_ids.size(); ++idx) {
      controller.set_speed_limit(motor_ids[idx], init_speed);
      controller.set_torque_limit(motor_ids[idx], LOW_GAIN_TORQUE_LIMIT);
      controller.set_current_limit(motor_ids[idx], LOW_GAIN_CURRENT_LIMIT);
      controller.set_position_control_gain(motor_ids[idx], LOW_GAIN_POSITION_KP);
      controller.set_velocity_control_gain(motor_ids[idx], LOW_GAIN_VELOCITY_KP, LOW_GAIN_VELOCITY_KI);
      controller.set_current_control_param(motor_ids[idx], LOW_GAIN_CURRENT_KP, LOW_GAIN_CURRENT_KI, LOW_GAIN_CURRENT_GAIN);
    }
    controller.enable_motors();
    mode = State::LOW_GAIN;

  } else if (mode == State::LOW_GAIN) {
    for (uint8_t idx = 0; idx < motor_ids.size(); ++idx) {
      controller.send_position_command(motor_ids[idx], init_pose[idx]);
      controller.process_can_packet();
    }

  } else if (mode == State::CHANGE_TO_HIGH_GAIN) {
    M5.Lcd.fillScreen(RED);
    M5.Lcd.setTextColor(WHITE, RED);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextSize(4);
    M5.Lcd.setTextDatum(7);
    M5.Lcd.drawString("HighGain Mode", M5.Lcd.width() / 2, M5.Lcd.height() / 2, 1);

    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextDatum(7);
    M5.Lcd.drawString("Stop", M5.Lcd.width() / 2 - M5.Lcd.width() / 3, M5.Lcd.height() - 10, 1);
    M5.Lcd.drawString("Low", M5.Lcd.width() / 2, M5.Lcd.height() - 10, 1);
    M5.Lcd.drawString("Motion", M5.Lcd.width() / 2 + M5.Lcd.width() / 3, M5.Lcd.height() - 10, 1);
    // M5.Lcd.drawString("Calib", M5.Lcd.width() / 2 + M5.Lcd.width() / 3, M5.Lcd.height() - 10, 1);

    controller.reset_motors();
    controller.set_run_mode(MODE_POSITION);
    for (uint8_t idx = 0; idx < motor_ids.size(); ++idx) {
      controller.set_speed_limit(motor_ids[idx], move_speed);
      controller.set_torque_limit(motor_ids[idx], HIGH_GAIN_TORQUE_LIMIT);
      controller.set_current_limit(motor_ids[idx], HIGH_GAIN_CURRENT_LIMIT);
      controller.set_position_control_gain(motor_ids[idx], HIGH_GAIN_POSITION_KP);
      controller.set_velocity_control_gain(motor_ids[idx], HIGH_GAIN_VELOCITY_KP, HIGH_GAIN_VELOCITY_KI);
      controller.set_current_control_param(motor_ids[idx], HIGH_GAIN_CURRENT_KP, HIGH_GAIN_CURRENT_KI, HIGH_GAIN_CURRENT_GAIN);
    }
    controller.enable_motors();
    mode = State::HIGH_GAIN;

  } else if (mode == State::HIGH_GAIN) {
    for (uint8_t idx = 0; idx < motor_ids.size(); ++idx) {
      controller.send_position_command(motor_ids[idx], init_pose[idx]);
      controller.process_can_packet();
    }

  } else if (mode == State::CHANGE_TO_EXECUTE_MOTION) {
    M5.Lcd.fillScreen(RED);
    M5.Lcd.setTextColor(WHITE, RED);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextSize(4);
    M5.Lcd.setTextDatum(7);
    M5.Lcd.drawString("Demo Mode", M5.Lcd.width() / 2, M5.Lcd.height() / 2, 1);

    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextDatum(7);
    // M5.Lcd.drawString("Stop", M5.Lcd.width() / 2 - M5.Lcd.width() / 3, M5.Lcd.height() - 10, 1);
    // M5.Lcd.drawString("Low", M5.Lcd.width() / 2, M5.Lcd.height() - 10, 1);
    // M5.Lcd.drawString("Motion", M5.Lcd.width() / 2 + M5.Lcd.width() / 3, M5.Lcd.height() - 10, 1);
    // M5.Lcd.drawString("Calib", M5.Lcd.width() / 2 + M5.Lcd.width() / 3, M5.Lcd.height() - 10, 1);
    mode = State::EXECUTE_MOTION;

  } else if (mode == State::EXECUTE_MOTION) {
    demo_mode();
    mode = State::CHANGE_TO_HIGH_GAIN;
  }
}

void loop()
{
  M5.update();
  change_state();
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

    while (std::abs(std::accumulate(efforts.begin(), efforts.end(), 0.0) / efforts.size()) < 0.4f || efforts.size() < 40) {

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


void execute_motion(const std::vector<Motion>& motion_seq)
{
  for (size_t motion_idx = 0; motion_idx < motion_seq.size(); ++motion_idx) {
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
}


void display_motor_state()
{
  controller.send_current_command(motor_ids, {0.0, 0.0, 0.0});
  std::vector<MotorStatus> status_list;
  if (controller.process_can_packet()) {
    if (controller.get_motor_status(status_list)) {
      uint16_t bg_color = BLACK;
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


void demo_mode()
{
  // y-z plane circle
  {
    std::vector<Motion> motion_seq;
    for (uint8_t cnt = 0; cnt < 5; ++cnt) {
      float duration = 0.004f + (4 - cnt) * 0.004f;
      for (uint8_t idx = 0; idx < 100; ++idx) {
        float j1, j2, j3;
        float x = 0.20;
        float y = 0.05 * std::cos(2 * M_PI / 100 * idx);
        float z = 0.05 * std::sin(2 * M_PI / 100 * idx);
        calc_inverse_kinematics(x, y, z, j1, j2, j3);
        Motion mot = {duration, {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3}, {
          {j1, 3.0f, 1.0f, 1.0f},
          {j2, 3.0f, 1.0f, 1.0f},
          {j3, 3.0f, 1.0f, 1.0f},
        }};
        motion_seq.push_back(mot);
      }
    }
    execute_motion(motion_seq);
  }

  // x-y plane circle
  {
    std::vector<Motion> motion_seq;
    for (uint8_t cnt = 0; cnt < 5; ++cnt) {
      float duration = 0.004f + (4 - cnt) * 0.004f;
      for (uint8_t idx = 0; idx < 100; ++idx) {
        float j1, j2, j3;
        float x = 0.05 * std::cos(2 * M_PI / 100 * idx) + 0.20;
        float y = 0.05 * std::sin(2 * M_PI / 100 * idx);
        float z = 0.1;
        calc_inverse_kinematics(x, y, z, j1, j2, j3);
        Motion mot = {duration, {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3}, {
          {j1, 3.0f, 10.0f, 1.0f, 1.0f},
          {j2, 3.0f, 10.0f, 1.0f, 1.0f},
          {j3, 3.0f, 10.0f, 1.0f, 1.0f},
        }};
        motion_seq.push_back(mot);
      }
    }
    execute_motion(motion_seq);
  }

  // x-z plane circle
  {
    std::vector<Motion> motion_seq;
    for (uint8_t cnt = 0; cnt < 5; ++cnt) {
      float duration = 0.004f + (4 - cnt) * 0.004f;
      for (uint8_t idx = 0; idx < 100; ++idx) {
        float j1, j2, j3;
        float x = 0.05 * std::cos(2 * M_PI / 100 * idx) + 0.2;
        float y = 0.0;
        float z = 0.05 * std::sin(2 * M_PI / 100 * idx);
        calc_inverse_kinematics(x, y, z, j1, j2, j3);
        Motion mot = {duration, {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3}, {
          {j1, 3.0f, 10.0f, 1.0f, 1.0f},
          {j2, 3.0f, 10.0f, 1.0f, 1.0f},
          {j3, 3.0f, 10.0f, 1.0f, 1.0f},
        }};
        motion_seq.push_back(mot);
      }
    }
    execute_motion(motion_seq);
  }

  // y-z plane
  {
    std::vector<Motion> motion_seq;
    for (uint8_t cnt = 0; cnt < 3; ++cnt) {
      float duration = 0.01f + (2 - cnt) * 0.01f;
      for (uint8_t idx = 0; idx < 100; ++idx) {
        float j1, j2, j3;
        float x = 0.2;
        float y = -0.15 + 0.3 / 100 * idx;
        float z = 0.05 * std::sin(8 * M_PI/100 * idx);
        calc_inverse_kinematics(x, y, z, j1, j2, j3);
        Motion mot = {duration, {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3}, {
          {j1, 0.0f, 10.0f, 1.0f, 1.0f},
          {j2, 0.0f, 10.0f, 1.0f, 1.0f},
          {j3, 0.0f, 10.0f, 1.0f, 1.0f},
        }};
        motion_seq.push_back(mot);
      }
      for (uint8_t idx = 0; idx < 100; ++idx) {
        float j1, j2, j3;
        float x = 0.2;
        float y = 0.15 - 0.3 / 100 * idx;
        float z = 0.05 * std::sin(8 * M_PI/100 * idx);
        calc_inverse_kinematics(x, y, z, j1, j2, j3);
        Motion mot = {duration, {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3}, {
          {j1, 0.0f, 10.0f, 1.0f, 1.0f},
          {j2, 0.0f, 10.0f, 1.0f, 1.0f},
          {j3, 0.0f, 10.0f, 1.0f, 1.0f},
        }};
        motion_seq.push_back(mot);
      }
    }
    execute_motion(motion_seq);
  }


  // set init pose
  {
    std::vector<Motion> motion_seq;
    Motion mot = {2.0f, {MOT_JOINT_1, MOT_JOINT_2, MOT_JOINT_3}, {
      {init_pose[0], 0.0f, 10.0f, 1.0f, 1.0f},
      {init_pose[1], 0.0f, 10.0f, 1.0f, 1.0f},
      {init_pose[2], 0.0f, 10.0f, 1.0f, 1.0f},
    }};
    motion_seq.push_back(mot);
    execute_motion(motion_seq);
  }

  controller.reset_motors();
  for (uint8_t idx = 0; idx < motor_ids.size(); ++idx) {
    controller.set_speed_limit(motor_ids[idx], init_speed);
  }
}
