#include "cybergear_controller.hh"
#include "cybergear_driver.hh"
#include "cybergear_driver_defs.hh"
#include <mcp_can.h>
#include <M5Stack.h>

CybergearController::CybergearController(uint8_t master_can_id)
  : can_(NULL)
  , master_can_id_(master_can_id)
{}

CybergearController::~CybergearController()
{}

bool CybergearController::init(const std::vector<uint8_t> ids, uint8_t mode, MCP_CAN* can)
{
  // setup variables
  can_ = can;
  motor_ids_ = ids;       // for send sequence
  control_mode_ = mode;

  // create motor class
  for (uint8_t idx = 0; idx < ids.size(); ++idx) {
    CybergearDriver driver = CybergearDriver(master_can_id_, ids[idx]);
    driver.init(can_);
    driver.init_motor(mode);
    drivers_[ids[idx]] = driver;
    motor_update_flag_[ids[idx]] = false;
  }

  return true;
}

bool CybergearController::set_motor_config(const std::vector<CybergearConfig>& configs)
{
  bool ret = true;
  for (uint8_t idx = 0; idx < configs.size(); ++idx) {
    ret &= set_motor_config(configs[idx]);
  }
  return ret;
}

bool CybergearController::set_motor_config(const CybergearConfig& config)
{
  if (!check_motor_id(config.id)) return false;

  // send config command
  drivers_[config.id].set_limit_speed(config.limit_speed);
  drivers_[config.id].set_limit_torque(config.limit_torque);
  drivers_[config.id].set_limit_current(config.limit_current);
  drivers_[config.id].set_current_kp(config.current_kp);
  drivers_[config.id].set_current_ki(config.current_ki);
  drivers_[config.id].set_current_filter_gain(config.current_filter_gain);
  return true;
}

bool CybergearController::set_speed_limit(uint8_t id, float limit)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].set_limit_speed(limit);
  return true;
}

bool CybergearController::set_torque_limit(uint8_t id, float limit)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].set_limit_torque(limit);
  return true;
}

bool CybergearController::set_current_limit(uint8_t id, float limit)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].set_limit_current(limit);
  return true;
}

bool CybergearController::set_position_control_gain(uint8_t id, float kp)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].set_position_kp(kp);
  return true;
}

bool CybergearController::set_velocity_control_gain(uint8_t id, float kp, float ki)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].set_velocity_kp(kp);
  drivers_[id].set_velocity_ki(ki);
  return true;
}

bool CybergearController::set_current_control_param(uint8_t id, float kp, float ki, float gain)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].set_current_kp(kp);
  drivers_[id].set_current_ki(ki);
  drivers_[id].set_current_filter_gain(gain);
  return true;
}

bool CybergearController::enable_motor(uint8_t id, uint8_t mode)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].init_motor(mode);
  drivers_[id].enable_motor();
  return true;
}

bool CybergearController::enable_motors()
{
  for (uint8_t idx; idx < motor_ids_.size(); ++idx) {
    drivers_[motor_ids_[idx]].enable_motor();
  }
  return true;
}

bool CybergearController::reset_motor(uint8_t id)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].reset_motor();
  return true;
}

bool CybergearController::reset_motors()
{
  for (uint8_t idx; idx < motor_ids_.size(); ++idx) {
    drivers_[motor_ids_[idx]].reset_motor();
  }
  return true;
}

bool CybergearController::send_motion_command(const std::vector<uint8_t> ids, const std::vector<CybergearMotionCommand> cmds)
{
  // check size
  if (ids.size() != cmds.size()) {
    return false;
  }

  bool ret = true;
  for (uint8_t idx = 0; idx < ids.size(); ++idx) {
    ret &= send_motion_command(ids[idx], cmds[idx]);
  }
  return ret;
}

bool CybergearController::send_motion_command(uint8_t id, const CybergearMotionCommand& cmd)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].motor_control(cmd.position, cmd.velocity, cmd.effort, cmd.kp, cmd.kd);
  return true;
}

bool CybergearController::send_position_command(const std::vector<uint8_t> ids, const std::vector<float> positions)
{
  // check size
  if (ids.size() != positions.size()) {
    return false;
  }

  bool ret = true;
  for (uint8_t idx = 0; idx < ids.size(); ++idx) {
    ret &= send_position_command(ids[idx], positions[idx]);
  }
  return ret;
}

bool CybergearController::send_position_command(uint8_t id, float position)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].set_position_ref(position);
  return true;
}

bool CybergearController::send_speed_command(const std::vector<uint8_t> ids, const std::vector<float> speeds)
{
  // check size
  if (ids.size() != speeds.size()) {
    return false;
  }

  bool ret = true;
  for (uint8_t idx = 0; idx < ids.size(); ++idx) {
    ret &= send_speed_command(ids[idx], speeds[idx]);
  }
  return ret;
}

bool CybergearController::send_speed_command(uint8_t id, float speed)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].set_speed_ref(speed);
  return true;
}

bool CybergearController::send_current_command(const std::vector<uint8_t> ids, const std::vector<float> currents)
{
  // check size
  if (ids.size() != currents.size()) {
    return false;
  }

  bool ret = true;
  for (uint8_t idx = 0; idx < ids.size(); ++idx) {
    ret &= send_current_command(ids[idx], currents[idx]);
  }
  return ret;
}

bool CybergearController::send_current_command(uint8_t id, float current)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].set_current_ref(current);
  return true;
}

bool CybergearController::set_mech_position_to_zero(uint8_t id)
{
  if (!check_motor_id(id)) return false;
  drivers_[id].set_mech_position_to_zero();
  return true;
}

bool CybergearController::get_motor_status(std::vector<MotorStatus> & status)
{
  for (uint8_t idx = 0; idx < motor_ids_.size(); ++idx) {
    MotorStatus mot = drivers_[motor_ids_[idx]].get_motor_status();
    status.push_back(mot);
  }
  return true;
}

bool CybergearController::get_motor_status(uint8_t id, MotorStatus& status)
{
  if (!check_motor_id(id)) return false;
  status = drivers_[id].get_motor_status();
  return true;
}

bool CybergearController::process_can_packet()
{
  bool is_updated = false;

  while ( can_->checkReceive() == CAN_MSGAVAIL ) {
    // receive data
    unsigned long id;
    uint8_t len;
    if (can_->readMsgBuf(&id, &len, receive_buffer_) == CAN_NOMSG) {
      break;
    }

    // if id is not mine
    uint8_t receive_can_id = id & 0xff;
    if ( receive_can_id != master_can_id_ ) {
      continue;
    }

    uint8_t motor_can_id = (id & 0xff00) >> 8;
    if (drivers_.find(motor_can_id) == drivers_.end()) {
      continue;
    }

    // parse packet --------------
    if (drivers_[motor_can_id].update_motor_status(id, receive_buffer_, len)) {
      motor_update_flag_[motor_can_id] = true;
      is_updated = true;
    }
  }

  return is_updated;
}

bool CybergearController::check_update_flag(uint8_t id)
{
  if (!check_motor_id(id)) return false;
  return motor_update_flag_[id];
}

bool CybergearController::reset_update_flag(uint8_t id)
{
  if (!check_motor_id(id)) return false;
  motor_update_flag_[id] = false;
  return true;
}

std::vector<uint8_t> CybergearController::motor_ids() const
{
  return motor_ids_;
}

bool CybergearController::check_motor_id(uint8_t id)
{
  CybergearDriverMap::iterator it = drivers_.find(id);
  if (it == drivers_.end()) {
    return false;
  }
  return true;
}
