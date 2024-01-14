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
#include "cybergear_controller.hh"


/**
 * @brief Init can interface
 */
void init_can();
void calibrate_positoin_zero_offset();

// init MCP_CAN object
#define CAN0_INT 15  // Set INT to pin 2
MCP_CAN CAN0(12);    // Set CS to pin 10

// setup master can id and motor can id (default cybergear can id is 0x7F)
const uint8_t MASTER_CAN_ID = 0x00;
const uint8_t MOT_JOINT_11 = 0x7F;    // 127
const uint8_t MOT_JOINT_12 = 0x7E;    // 126
const uint8_t MOT_JOINT_13 = 0x7D;    // 125
const uint8_t MOT_JOINT_21 = 0x7C;    // 124
const uint8_t MOT_JOINT_22 = 0x7B;    // 123
const uint8_t MOT_JOINT_23 = 0x7A;    // 122
std::vector<uint8_t> motor_ids = {MOT_JOINT_11, MOT_JOINT_12, MOT_JOINT_13, MOT_JOINT_21, MOT_JOINT_22, MOT_JOINT_23};
std::vector<float> init_pose = {0.0f, M_PI/4, 0.0f, 0.0f, M_PI/4, 0.0f};
std::vector<CybergearSoftwareConfig> sw_configs = {
   // id, direction, limit_speed, limit_current, limit_torque, upper_pos_limit, lower_pos_limit, pos_offset
  // arm1
  CybergearSoftwareConfig(MOT_JOINT_11, CCW, 1.0f, 1.0f, 0.5f, (float)65.0/180.0*M_PI, (float)-65.0/180.0*M_PI, CCW, (float)-65.0/180.0*M_PI),
  CybergearSoftwareConfig(MOT_JOINT_12, CCW, 1.0f, 1.0f, 0.5f, (float)M_PI/2.0f, (float)-M_PI/4.0f, CW, (float)M_PI/2),
  CybergearSoftwareConfig(MOT_JOINT_13, CW,  1.0f, 1.0f, 0.5f, (float)M_PI, (float)-M_PI/4.0f, CCW, (float)-30.0/180.0*M_PI),
  // arm2
  CybergearSoftwareConfig(MOT_JOINT_21, CCW, 1.0f, 1.0f, 0.5f, (float)65.0/180.0*M_PI, (float)-65.0/180.0*M_PI, CCW, (float)-65.0/180.0*M_PI),
  CybergearSoftwareConfig(MOT_JOINT_22, CCW, 1.0f, 1.0f, 0.5f, (float)M_PI/2.0f, (float)-M_PI/4.0f, CW, (float)M_PI/2),
  CybergearSoftwareConfig(MOT_JOINT_23, CW,  1.0f, 1.0f, 0.5f, (float)M_PI, (float)-M_PI/4.0f, CCW, (float)-30.0/180.0*M_PI)
};

// state
#define ORIGIN_DETECT_SPEED     1.2f
#define INITIAL_POSE_MOVE_SPEED 1.8f
std::vector<uint8_t> calib_motor_seq = {MOT_JOINT_13, MOT_JOINT_12, MOT_JOINT_11, MOT_JOINT_23, MOT_JOINT_22, MOT_JOINT_21};

// birateral settings
const float LOW_GAIN_BIRATERAL_GAIN = 3.0;
const float HIGH_GAIN_BIRATERAL_GAIN = 25.0;
float birateral_gain = LOW_GAIN_BIRATERAL_GAIN;
const float MAX_CURRENT = 2.0;
std::vector<float> birateral_currents = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

enum struct State
{
  INIT,
  STOP,
  CALIBRATE_ORIGIN,
  CHANGE_TO_LOW_GAIN,
  LOW_GAIN,
  CHANGE_TO_HIGH_GAIN,
  HIGH_GAIN,
  ERROR,
};

State mode = State::INIT;

// init cybergeardriver
CybergearController controller = CybergearController(MASTER_CAN_ID);

const float init_speed = 1.0f;        //!< slow speed
const float move_speed = 10.0f;       //!< quick speed


void setup()
{
  M5.begin(true,true,false);
  // Serial.begin(2000000);
  Serial.begin(115200);
  Serial.flush();

  // init cybergear driver
  init_can();

  controller.init(motor_ids, sw_configs, MODE_CURRENT, &CAN0);

  for (uint8_t idx = 0; idx < motor_ids.size(); ++idx) {
    controller.set_speed_limit(motor_ids[idx], init_speed);
    controller.set_torque_limit(motor_ids[idx], 12.0f);
    controller.set_current_limit(motor_ids[idx], 10.0f);
  }

  delay(1000);
  M5.Lcd.fillScreen(BLACK);
}

void change_state()
{
  // state change event
  if (M5.BtnA.wasPressed()) {
    if (mode == State::LOW_GAIN) {
      mode = State::INIT;
    }
    else if (mode == State::HIGH_GAIN) {
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
    // M5.Lcd.drawString("Stop", M5.Lcd.width() / 2 - M5.Lcd.width() / 3, M5.Lcd.height() - 10, 1);
    M5.Lcd.drawString("Low", M5.Lcd.width() / 2, M5.Lcd.height() - 10, 1);
    // M5.Lcd.drawString("High", M5.Lcd.width() / 2 + M5.Lcd.width() / 3, M5.Lcd.height() - 10, 1);
    M5.Lcd.drawString("Calib", M5.Lcd.width() / 2 + M5.Lcd.width() / 3, M5.Lcd.height() - 10, 1);

    controller.reset_motors();
    birateral_currents = std::vector<float>(motor_ids.size(), 0.0f);
    mode = State::STOP;

  } else if (mode == State::STOP) {
    // set zero torque
    // DO NOTHING

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
    controller.set_run_mode(MODE_CURRENT);
    for (uint8_t idx = 0; idx < motor_ids.size(); ++idx) {
      controller.set_speed_limit(motor_ids[idx], init_speed);
    }
    controller.enable_motors();
    std::vector<float> current_commands(motor_ids.size(), 0.0f);
    controller.send_current_command(motor_ids, current_commands);

    birateral_gain = LOW_GAIN_BIRATERAL_GAIN;
    mode = State::LOW_GAIN;

  } else if (mode == State::LOW_GAIN) {
    birateral_mode();

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
    // M5.Lcd.drawString("High", M5.Lcd.width() / 2 + M5.Lcd.width() / 3, M5.Lcd.height() - 10, 1);
    // M5.Lcd.drawString("Calib", M5.Lcd.width() / 2 + M5.Lcd.width() / 3, M5.Lcd.height() - 10, 1);

    controller.reset_motors();
    controller.set_run_mode(MODE_CURRENT);
    for (uint8_t idx = 0; idx < motor_ids.size(); ++idx) {
      controller.set_speed_limit(motor_ids[idx], move_speed);
    }

    controller.enable_motors();
    std::vector<float> current_commands(motor_ids.size(), 0.0f);
    controller.send_current_command(motor_ids, current_commands);

    birateral_gain = HIGH_GAIN_BIRATERAL_GAIN;
    mode = State::HIGH_GAIN;

  } else if (mode == State::HIGH_GAIN) {
    birateral_mode();
  }
}

void loop()
{
  // update m5 satatus
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

    while (std::abs(std::accumulate(efforts.begin(), efforts.end(), 0.0) / efforts.size()) < 0.6f || efforts.size() < 40) {

      // set motor speed
      controller.send_speed_command(calib_motor_seq[idx], config.calib_direction * ORIGIN_DETECT_SPEED);
      if (controller.process_can_packet()) {
        MotorStatus mot;
        controller.get_motor_status(calib_motor_seq[idx], mot);
        efforts.push_back(mot.effort);
        Serial.printf("Update data %f\n", mot.effort);
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
  std::vector<float> pos_diffs(motor_ids.size(), 1000.0f);
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


void birateral_mode()
{
  // collect motor data
  controller.send_current_command(motor_ids, birateral_currents);

  // check motor state
  std::vector<MotorStatus> status_list;
  if (controller.get_motor_status(status_list)) {
    // calc birateral outputs
    birateral_currents[0] = (status_list[3].position - status_list[0].position) * birateral_gain;
    birateral_currents[3] = (status_list[0].position - status_list[3].position) * birateral_gain;

    birateral_currents[1] = (status_list[4].position - status_list[1].position) * birateral_gain;
    birateral_currents[4] = (status_list[1].position - status_list[4].position) * birateral_gain;

    birateral_currents[2] = (status_list[5].position - status_list[2].position) * birateral_gain;
    birateral_currents[5] = (status_list[2].position - status_list[5].position) * birateral_gain;

    // clamp data range
    for (uint8_t idx = 0; idx < birateral_currents.size(); ++idx) {
      birateral_currents[idx] = min(max(birateral_currents[idx], -MAX_CURRENT), MAX_CURRENT);
    }
  }
}
