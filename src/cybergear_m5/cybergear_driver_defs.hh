#ifndef CYBER_GEAR_DRIVER_DEFS_H
#define CYBER_GEAR_DRIVER_DEFS_H

#include <cstdint>

static constexpr uint8_t CMD_POSITION = 1;
static constexpr uint8_t CMD_RESPONSE = 2;
static constexpr uint8_t CMD_ENABLE = 3;
static constexpr uint8_t CMD_RESET = 4;
static constexpr uint8_t CMD_SET_MECH_POSITION_TO_ZERO = 6;
static constexpr uint8_t CMD_CHANGE_CAN_ID = 7;
static constexpr uint8_t CMD_RAM_READ = 17;
static constexpr uint8_t CMD_RAM_WRITE = 18;
static constexpr uint8_t CMD_GET_MOTOR_FAIL = 21;

static constexpr uint16_t ADDR_RUN_MODE = 0x7005;
static constexpr uint16_t ADDR_IQ_REF = 0x7006;
static constexpr uint16_t ADDR_SPEED_REF = 0x700A;
static constexpr uint16_t ADDR_LIMIT_TORQUE = 0x700B;
static constexpr uint16_t ADDR_CURRENT_KP = 0x7010;
static constexpr uint16_t ADDR_CURRENT_KI = 0x7011;
static constexpr uint16_t ADDR_CURRENT_FILTER_GAIN = 0x7014;
static constexpr uint16_t ADDR_LOC_REF = 0x7016;
static constexpr uint16_t ADDR_LIMIT_SPEED = 0x7017;
static constexpr uint16_t ADDR_LIMIT_CURRENT = 0x7018;
static constexpr uint16_t ADDR_MECH_POS = 0x7019;
static constexpr uint16_t ADDR_IQF = 0x701A;
static constexpr uint16_t ADDR_MECH_VEL = 0x701B;
static constexpr uint16_t ADDR_VBUS = 0x701C;
static constexpr uint16_t ADDR_ROTATION = 0x701D;
static constexpr uint16_t ADDR_LOC_KP = 0x701E;
static constexpr uint16_t ADDR_SPD_KP = 0x701F;
static constexpr uint16_t ADDR_SPD_KI = 0x7020;

static constexpr uint8_t MODE_MOTION = 0x00;
static constexpr uint8_t MODE_POSITION = 0x01;
static constexpr uint8_t MODE_SPEED = 0x02;
static constexpr uint8_t MODE_CURRENT = 0x03;

static constexpr float P_MIN = -12.5f;
static constexpr float P_MAX = 12.5f;
static constexpr float V_MIN = -30.0f;
static constexpr float V_MAX = 30.0f;
static constexpr float KP_MIN = 0.0f;
static constexpr float KP_MAX = 500.0f;
static constexpr float KD_MIN = 0.0f;
static constexpr float KD_MAX = 5.0f;
static constexpr float T_MIN = -12.0f;
static constexpr float T_MAX = 12.0f;
static constexpr float IQ_MIN = -27.0f;
static constexpr float IQ_MAX = 27.0f;
static constexpr float CURRENT_FILTER_GAIN_MIN = 0.0f;
static constexpr float CURRENT_FILTER_GAIN_MAX = 1.0f;

static constexpr float IQ_REF_MAX = 23.0f;
static constexpr float IQ_REF_MIN = -23.0f;
static constexpr float SPD_REF_MAX = 30.0f;
static constexpr float SPD_REF_MIN = -30.0f;
static constexpr float LIMIT_TORQUE_MAX = 12.0f;
static constexpr float LIMIT_TORQUE_MIN = 0.0f;
static constexpr float CUR_KP_MAX = 200.0f;
static constexpr float CUR_KP_MIN = 0.0f;
static constexpr float CUR_KI_MAX = 200.0f;
static constexpr float CUR_KI_MIN = 0.0f;
static constexpr float LOC_KP_MAX = 200.0f;
static constexpr float LOC_KP_MIN = 0.0f;
static constexpr float SPD_KP_MAX = 200.0f;
static constexpr float SPD_KP_MIN = 0.0f;
static constexpr float LIMIT_SPD_MAX = 30.0f;
static constexpr float LIMIT_SPD_MIN = 0.0f;
static constexpr float LIMIT_CURRENT_MAX = 27.0f;
static constexpr float LIMIT_CURRENT_MIN = 0.0f;

static constexpr float DEFAULT_CURRENT_KP = 0.125f;
static constexpr float DEFAULT_CURRENT_KI = 0.0158f;
static constexpr float DEFAULT_CURRENT_FINTER_GAIN = 0.1f;
static constexpr float DEFAULT_POSITION_KP = 30.0f;
static constexpr float DEFAULT_VELOCITY_KP = 2.0f;
static constexpr float DEFAULT_VELOCITY_KI = 0.002f;
static constexpr float DEFAULT_VELOCITY_LIMIT = 2.0f;
static constexpr float DEFAULT_CURRENT_LIMIT = 27.0f;
static constexpr float DEFAULT_TORQUE_LIMIT = 12.0f;

static constexpr uint8_t RET_CYBERGEAR_OK = 0x00;
static constexpr uint8_t RET_CYBERGEAR_MSG_NOT_AVAIL = 0x01;
static constexpr uint8_t RET_CYBERGEAR_INVALID_CAN_ID = 0x02;
static constexpr uint8_t RET_CYBERGEAR_INVALID_PACKET = 0x03;

static constexpr uint16_t CYBERGEAR_RESPONSE_TIME_USEC = 250;

#define CW 1
#define CCW -1

#endif  // !CYBER_GEAR_DRIVER_DEFS_H
