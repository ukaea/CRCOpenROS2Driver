#include "crcopen_hardware.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace crcopen_hardware
{

  constexpr const char *LOG_NAME = "CRCOpenHardware";

  void CRCOpenHardware::orld_thread_function()
  {
    while (keep_thread_running)
    {
      // READ
      if (ORLD_CycleRead(ORL_VERB_OFF, ORL_CNTRL1) != ORL_OK)
      {
        RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_NAME), "CYCLEREAD NOT OK");
        break;
      };

      for (Joint* joint: open_joints_)
      {
        uint axis_idx = joint->axis_idx;
        ORLD_GetMeasuredPosMR(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, axis_idx, &orld_pos_meas[axis_idx]);
        ORLD_GetMeasuredSpeedMRxStep(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, axis_idx, &orld_vel_meas[axis_idx]);
        ORLD_GetMeasuredCurrent(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, axis_idx, &orld_cur_meas[axis_idx]);
        // option here of setting ruckig current state to read state - didn't work in testing so ruckig operates 'blind'
      }

      // INTERPOLATE (relevant for 'position', 'velocity', 'acceleration' control modes)
      ruckig_output.pass_to_input(ruckig_input);
      int res = ruckig_otg.update(ruckig_input, ruckig_output);

      // WRITE
      for (Joint* joint: open_joints_)
      {
        uint axis_idx = joint->axis_idx;
        switch (control_mode)
        {
        case ControlMode::LISTEN:
          break;
        case ControlMode::POSITION:
          // Use ruckig interpolation and convert from rad to motorround
          orld_pos_target[axis_idx] = rad_to_crc_pos(ruckig_output.new_position[axis_idx], *joint);
          ORLD_SetTargetPosMR(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, axis_idx, orld_pos_target[axis_idx]);
          break;
        case ControlMode::POSITION_DIRECT:
          // Use ros2control command and convert from rad to motorround
          orld_pos_target[axis_idx] = rad_to_crc_pos(joint->command.position, *joint);
          ORLD_SetTargetPosMR(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, axis_idx, orld_pos_target[axis_idx]);
          break;
        case ControlMode::VELOCITY:
        case ControlMode::ACCELERATION:
          // Use ruckig interpolation for velocity and convert from rad/s to motorround/cycle
          orld_vel_target[axis_idx] = rad_to_crc_vel(ruckig_output.new_velocity[axis_idx], *joint);
          // Use ruckig interpolation for position target and convert from rad to motorround           
          orld_pos_target[axis_idx] = rad_to_crc_pos(ruckig_output.new_position[axis_idx], *joint);
          ORLD_SetTargetPosMR(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, axis_idx, orld_pos_target[axis_idx]);
          ORLD_SetTargetSpeedMRxStep(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, axis_idx, orld_vel_target[axis_idx]);
          break;
        case ControlMode::VELOCITY_POSITION_DIRECT:
          // Use ros2control commands and convert to motorround
          orld_vel_target[axis_idx] = rad_to_crc_vel(joint->command.velocity, *joint);             
          orld_pos_target[axis_idx] = rad_to_crc_pos(joint->command.position, *joint);
          ORLD_SetTargetPosMR(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, axis_idx, orld_pos_target[axis_idx]);
          ORLD_SetTargetSpeedMRxStep(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, axis_idx, orld_vel_target[axis_idx]);
          break;
        case ControlMode::CURRENT:
          // Use ros2control command
          orld_cur_target[axis_idx] = joint->command.current;
           // Cheat position following check by assuming no change in velocity (vel is in mr/cycle so just adding gives movement in one cycle)
          orld_pos_target[axis_idx] = orld_pos_meas[axis_idx] + orld_vel_meas[axis_idx];
          ORLD_SetTargetPosMR(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, axis_idx, orld_pos_target[axis_idx]);
          ORLD_SetCurrentContribution(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, axis_idx, orld_cur_target[axis_idx]);
          break;
        case ControlMode::CURRENT_POSITION_DIRECT:
          // Use ros2control command
          orld_cur_target[axis_idx] = joint->command.current;
          // Use ros2control command and convert from rad to motorround
          orld_pos_target[axis_idx] = rad_to_crc_pos(joint->command.position, *joint);
          ORLD_SetTargetPosMR(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, axis_idx, orld_pos_target[axis_idx]);
          ORLD_SetCurrentContribution(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, axis_idx, orld_cur_target[axis_idx]);
          break;
        case ControlMode::TORQUE:
          orld_cur_target[axis_idx] = trq_to_cur(joint->command.torque, *joint);
          orld_pos_target[axis_idx] = orld_pos_meas[axis_idx] + orld_vel_meas[axis_idx];
          ORLD_SetTargetPosMR(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, axis_idx, orld_pos_target[axis_idx]);
          ORLD_SetCurrentContribution(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, axis_idx, orld_cur_target[axis_idx]);
          break;
        case ControlMode::TORQUE_POSITION_DIRECT:
          orld_cur_target[axis_idx] = trq_to_cur(joint->command.torque, *joint);
          orld_pos_target[axis_idx] = rad_to_crc_pos(joint->command.position, *joint);
          ORLD_SetTargetPosMR(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, axis_idx, orld_pos_target[axis_idx]);
          ORLD_SetCurrentContribution(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, axis_idx, orld_cur_target[axis_idx]);
          break;
        }
      }
      
      if (ORLD_CycleWrite(ORL_VERB_OFF, ORL_CNTRL1) != ORL_OK){
        RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_NAME), "CYCLEWRITE NOT OK");
        break;
      };
    }
    return;
  }

  hardware_interface::CallbackReturn CRCOpenHardware::on_init(const hardware_interface::HardwareInfo &info)
  {
    RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "on_init");
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Get network address parameters
    try
    {
      LPC_addr = info_.hardware_parameters.at("lpc_addr").c_str();
      CRC_addr = info_.hardware_parameters.at("crc_addr").c_str();
    }
    catch (const std::out_of_range &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "One (or both) of 'lpc_addr' and 'crc_addr' is not specified in ros_control xacro");
      return CallbackReturn::FAILURE;
    }

    // Get payload parameters
    try
    {
      payload_mass = std::stof(info_.hardware_parameters.at("payload_mass"));
      payload_cog[0] = std::stof(info_.hardware_parameters.at("payload_cog_x"));
      payload_cog[1] = std::stof(info_.hardware_parameters.at("payload_cog_y"));
      payload_cog[2] = std::stof(info_.hardware_parameters.at("payload_cog_z"));
      payload_inertia[0] = std::stof(info_.hardware_parameters.at("payload_inertia_xx"));
      payload_inertia[1] = std::stof(info_.hardware_parameters.at("payload_inertia_xy"));
      payload_inertia[2] = std::stof(info_.hardware_parameters.at("payload_inertia_xz"));
      payload_inertia[3] = std::stof(info_.hardware_parameters.at("payload_inertia_yy"));
      payload_inertia[4] = std::stof(info_.hardware_parameters.at("payload_inertia_yz"));
      payload_inertia[5] = std::stof(info_.hardware_parameters.at("payload_inertia_zz"));
    }
    catch (const std::out_of_range &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Not all payload parameters are specified in ros_control xacro");
      return CallbackReturn::FAILURE;
    }

    // Configure Ruckig online trajectory generator (otg)
    ruckig_otg.delta_time = 0.0004; // CRCOpen requires update cycle time of 400us
    ruckig_input.synchronization = ruckig::Synchronization::None;
    ruckig_input.current_position = {0,0,0,0,0,0};
    ruckig_input.current_velocity = {0,0,0,0,0,0};
    ruckig_input.current_acceleration = {0,0,0,0,0,0};
    ruckig_input.target_position = {0,0,0,0,0,0};
    ruckig_input.target_velocity = {0,0,0,0,0,0};
    ruckig_input.target_acceleration = {0,0,0,0,0,0};
    ruckig_input.max_velocity = {1,1,1,1,1,1};
    ruckig_input.max_acceleration = {1,1,1,1,1,1};
    // ruckig_input.min_acceleration = {-1,-1,-1,-1,-1,-1};
    ruckig_input.max_jerk = {1,1,1,1,1,1};
    ruckig_otg.update(ruckig_input, ruckig_output);

    // Initialise joints_ array
    joints_.resize(info_.joints.size(), Joint());

    for (size_t i = 0; i < joints_.size(); i++)
    {
      // Get descriptive joint name
      joints_[i].name = info_.joints[i].name;

      // Get joint axis index
      try
      {
        joints_[i].axis_idx = std::stoi(info_.joints[i].parameters.at("axis_idx"));
      }
      catch (const std::out_of_range &e)
      {
        RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "axis_idx for joint '%s' not specified in ros_control xacro", info_.joints[i].name.c_str());
        return CallbackReturn::FAILURE;
      }
      
      // Get limits for ruckig
      try
      {
        ruckig_input.max_velocity[joints_[i].axis_idx]     = std::stod(info_.joints[i].parameters.at("ruckig_max_vel"));
        ruckig_input.max_jerk[joints_[i].axis_idx]         = std::stod(info_.joints[i].parameters.at("ruckig_max_jrk"));

        true_max_acceleration[joints_[i].axis_idx] = std::stod(info_.joints[i].parameters.at("ruckig_max_acc"));
        ruckig_input.max_acceleration[joints_[i].axis_idx] = true_max_acceleration[joints_[i].axis_idx];
        // ruckig_input.min_acceleration.value()[joints_[i].axis_idx] = -true_max_acceleration[joints_[i].axis_idx];
      }
      catch (const std::out_of_range &e)
      {
        RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "joint '%s' does not have all of ruckig_max_vel, ruckig_max_acc, ruckig_max_jrk specified in ros2_control xacro", info_.joints[i].name.c_str());
        return CallbackReturn::FAILURE;
      }

      // Fill joint state and command with defaults
      joints_[i].state.position       = std::numeric_limits<double>::quiet_NaN();
      joints_[i].state.velocity       = std::numeric_limits<double>::quiet_NaN();
      joints_[i].state.acceleration   = std::numeric_limits<double>::quiet_NaN();
      joints_[i].state.current        = std::numeric_limits<double>::quiet_NaN();
      joints_[i].state.torque         = std::numeric_limits<double>::quiet_NaN();
      joints_[i].command.position     = std::numeric_limits<double>::quiet_NaN();
      joints_[i].command.velocity     = std::numeric_limits<double>::quiet_NaN();
      joints_[i].command.acceleration = std::numeric_limits<double>::quiet_NaN();
      joints_[i].command.current      = std::numeric_limits<double>::quiet_NaN();
      joints_[i].command.torque       = std::numeric_limits<double>::quiet_NaN();
    }
    
    // Set default control mode
    control_mode = ControlMode::LISTEN;

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> CRCOpenHardware::export_state_interfaces()
  {
    RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "export_state_interfaces");
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < joints_.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          joints_[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          joints_[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joints_[i].name, hardware_interface::HW_IF_ACCELERATION, &joints_[i].state.acceleration));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          joints_[i].name, "current", &joints_[i].state.current));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          joints_[i].name, "torque", &joints_[i].state.torque));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> CRCOpenHardware::export_command_interfaces()
  {
    RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "export_command_interfaces");
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < joints_.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          joints_[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          joints_[i].name, "position_direct", &joints_[i].command.position));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          joints_[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joints_[i].name, hardware_interface::HW_IF_ACCELERATION, &joints_[i].command.acceleration));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          joints_[i].name, "current", &joints_[i].command.current));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          joints_[i].name, "torque", &joints_[i].command.torque));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn CRCOpenHardware::on_configure(const rclcpp_lifecycle::State & /* previous_state */)
  {
    RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "on_configure");
    int err_status = ORL_ERR;

    // Initialise connection with CRC
    err_status = ORLD_Initialize(ORL_VERB_OFF, ORL_CNTRL1, LPC_addr, CRC_addr);
    keep_thread_running = true;
    orld_thread_handle = std::thread(&CRCOpenHardware::orld_thread_function, this);

    if (err_status != ORL_OK)
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Failed to initialise CRC controller: %d", err_status);
      return hardware_interface::CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "Connection initialized with CRC Controller.");

    // Small delay after intialising connection to allow spin to settle
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Set payload parameters
    err_status = ORLD_SrvSetPayload(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, payload_mass, payload_cog, payload_inertia);
    if (err_status == ORL_OK)
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_NAME), "Payload parameters set."
        << " m = "<<payload_mass
        << " CoG = "<<payload_cog[0]<<","<<payload_cog[1]<<","<<payload_cog[2]
        << " I = "<<payload_inertia[0]<<","<<payload_inertia[1]<<","<<payload_inertia[2]<<","<<payload_inertia[3]<<","<<payload_inertia[4]<<","<<payload_inertia[5]
        );
    } else {
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Failed to set payload: %d", err_status);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Get calibration paramters
    float tx_rate[ORL_AXIS_MAX];
    float cal_data[ORL_AXIS_MAX];
    float ax_infl[ORL_AXIS_MAX];
    err_status = ORLD_SrvGetKinematicData(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, tx_rate, cal_data, ax_infl);

    for (size_t i = 0; i < joints_.size(); i++)
    {
      uint axis_idx = joints_[i].axis_idx;
      joints_[i].tx_rate = tx_rate[axis_idx];
      joints_[i].cal_data = cal_data[axis_idx];
      if (ax_infl[i] != 0){
        RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Non-zero axis influence value detected in robot kinematics. Robots of this type are currently unsupported.");
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_NAME), "Robot calibration paramters fetched and set.");

    // Check drives are OFF
    int StatusArm;
    ORLD_GetStatusArm(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, &StatusArm);
    if (StatusArm == ORL_DRIVEON)
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Robot has DRIVE ON upon connection.");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Check which axes are open
    open_joints_.clear();
    for (size_t i = 0; i < joints_.size(); i++)
    {
      uint axis_idx = joints_[i].axis_idx;
      // Check joint claimed in xacro is open
      int AxisOpenness;
      ORLD_CheckOpenness(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, axis_idx, &AxisOpenness);
      if (AxisOpenness != ORL_AXISOPEN)
      {
        RCLCPP_WARN(rclcpp::get_logger(LOG_NAME), "Joint index %d, '%s', is CLOSED.", axis_idx, joints_[i].name.c_str());
        // return hardware_interface::CallbackReturn::ERROR;
      }
      else
      {
        RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "Joint index %d, '%s', is OPEN.", axis_idx, joints_[i].name.c_str());
        open_axes_mask |= (1 << axis_idx);
        open_joints_.push_back(&joints_[i]);
      }
      // Check axis starts in LISTEN modality
      int AxisMode;
      ORLD_GetModeAx(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, axis_idx, &AxisMode);
      if (AxisMode != ORL_LISTEN)
      {
        RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Joint %d, '%s', was not in LISTEN modality upon connection.", axis_idx, joints_[i].name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      };
    }

    // Attempt a reset of TP errors (eg previous CRCOpen disconnect)
    RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "Resetting TP error status.");
    ORLD_SrvErrorReset(ORL_VERB_OFF, ORL_CNTRL1);
    // rclcpp::sleep_for(std::chrono::seconds(1));

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn CRCOpenHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
  {
    RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "on_activate");
    int err_status = ORL_ERR;

    // Check Modal Selector Switch
    int modalStatus;
    err_status = ORLD_SrvGetModalStatus(ORL_VERB_OFF, ORL_CNTRL1, &modalStatus);

    if (err_status != ORL_OK)
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Failed to get modal status: %d", err_status);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (modalStatus != ORL_MODSTS_AL)
    {
      RCLCPP_WARN(rclcpp::get_logger(LOG_NAME), "Teach pendant in manual/T1 or AR mode. To enable remote control the switch must be in the AUTOTP position.");
      return hardware_interface::CallbackReturn::FAILURE;
    }

    // Call for drives on
    int StatusArm;
    ORLD_GetStatusArm(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, &StatusArm);

    if (StatusArm == ORL_DRIVEON)
    {
      RCLCPP_WARN(rclcpp::get_logger(LOG_NAME), "Drives already on when calling on_activate()");
    }
    else
    {
      ORLD_SrvSetDrive(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, ORL_DRIVEON);
      RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "ORLD_SrvSetDrive ON called.");
    }

    // Wait for drives on with a timeout
    auto start_time = rclcpp::Clock().now();
    while (StatusArm != ORL_DRIVEON)
    {
      ORLD_GetStatusArm(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, &StatusArm);

      // Check if 3 seconds have passed
      if ((rclcpp::Clock().now() - start_time).seconds() > 3.0)
      {
        RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Timeout waiting for DRIVES ON");
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "DRIVES ON");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn CRCOpenHardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
  {
    RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "on_deactivate");
    
    // Set axes to LISTEN
    int AxisMode;
    ORLD_GetModeMasterAx(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, &AxisMode);
    if (AxisMode != ORL_LISTEN)
    {
      RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "Switching to LISTEN modality");
      ORLD_SwitchToListenModality(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1);

      while (AxisMode != ORL_LISTEN)
      {
        ORLD_GetModeMasterAx(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, &AxisMode);
      }
    };
    RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "Drives in LISTEN modality.");

    // Small delay for movement to settle (if any)
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Perform Drive OFF
    int StatusArm;
    ORLD_GetStatusArm(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, &StatusArm);
    if (StatusArm == ORL_DRIVEON)
    {
      ORLD_SrvSetDrive(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, ORL_DRIVEOFF);
      RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "ORLD_SrvSetDrive OFF called.");
    }
    while (StatusArm != ORL_DRIVEOFF)
    {
      ORLD_GetStatusArm(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, &StatusArm);
      using namespace std::chrono_literals;
    }

    RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "DRIVES OFF");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn CRCOpenHardware::on_cleanup(const rclcpp_lifecycle::State & /* previous_state */)
  {
    RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "on_cleanup");
    ORLD_Close(ORL_VERB_OFF, ORL_CNTRL1);

    RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "Waiting for thread to end.");
    keep_thread_running = false;
    if (orld_thread_handle.joinable()){
      orld_thread_handle.join();
    }
    RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "Thread ended.");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn CRCOpenHardware::on_shutdown(const rclcpp_lifecycle::State & /* previous_state */)
  {
    RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "on_shutdown");
    on_deactivate(rclcpp_lifecycle::State());
    on_cleanup(rclcpp_lifecycle::State());
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type CRCOpenHardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  {
    // Copy and convert orld values into ros2control values.
    for (Joint* joint: open_joints_)
    {
      uint axis_idx = joint->axis_idx;
      joint->state.position = crc_to_rad_pos(orld_pos_meas[axis_idx], *joint); 
      joint->state.velocity = crc_to_rad_vel(orld_vel_meas[axis_idx], *joint);
      joint->state.current = orld_cur_meas[axis_idx];
      joint->state.torque = cur_to_trq(orld_cur_meas[axis_idx], *joint);

      joint->state.acceleration = ruckig_output.new_acceleration[axis_idx]; // Not real measurement but approximately valid for ruckig interpolated trajectories
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type CRCOpenHardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  { 
    // Update ruckig interpolation target as appropriate.
    for (Joint* joint: open_joints_)
    {
      uint axis_idx = joint->axis_idx;
      switch (control_mode) {
        case ControlMode::POSITION:
          ruckig_input.target_position[axis_idx] = joint->command.position;
          break;
        case ControlMode::VELOCITY:
          ruckig_input.target_velocity[axis_idx] = joint->command.velocity;
          break;
        case ControlMode::ACCELERATION:
          {
            // Clamp target to ruckig maximums given in urdf
            double acc_target = std::max(std::min(joint->command.acceleration, true_max_acceleration[axis_idx]), -true_max_acceleration[axis_idx]);
            if (acc_target > 0 ){
              ruckig_input.target_velocity[axis_idx] = ruckig_input.max_velocity[axis_idx];
              ruckig_input.max_acceleration[axis_idx] = acc_target;
              // ruckig_input.min_acceleration.value()[axis_idx] = -true_max_acceleration[axis_idx];
            } else if (acc_target < 0) {
              ruckig_input.target_velocity[axis_idx] = -ruckig_input.max_velocity[axis_idx];
              // ruckig_input.min_acceleration.value()[axis_idx] = -acc_target;
              // ruckig_input.max_acceleration[axis_idx] = true_max_acceleration[axis_idx];
              ruckig_input.max_acceleration[axis_idx] = -acc_target;
            } else {
              ruckig_input.target_velocity[axis_idx] = ruckig_input.current_velocity[axis_idx];
            }
            break;
          }
        default:
          break;
      }
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type CRCOpenHardware::prepare_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces)
  {
    RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "prepare_command_mode_switch");

    next_control_mode = control_mode;
    // If any relevant stop_interfaces, default to LISTEN
    for (const auto &key : stop_interfaces){
      for (size_t i = 0; i < joints_.size(); i++)
      {
        if (key.substr(0, key.find_last_of('/')) == joints_[i].name)
        {
          // There is a stopping interface belonging to the hardware
          next_control_mode = ControlMode::LISTEN;
        }
      }
    }
    
    uint start_position_direct_count = 0;
    std::vector<std::string> start_command_modes = {};

    // For each requested interface
    for (const auto &key : start_interfaces)
    {
      std::string requested_joint_name = key.substr(0,key.find_last_of('/'));
      std::string requested_joint_mode = key.substr(key.find_last_of('/')+1);
      bool requested_joint_is_open = false;

      // Find requested joint in list of open joints
      for (Joint *joint : open_joints_) {
        if (joint->name == requested_joint_name) {
          requested_joint_is_open = true;
          if (requested_joint_mode == "position_direct") {
            start_position_direct_count += 1;
          }
          else {
            start_command_modes.push_back(requested_joint_mode);
          }
        }
      }
      if (!requested_joint_is_open) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_NAME), "Rejecting command mode switch: Joint is not in list of open joints: "<<requested_joint_name);
        return hardware_interface::return_type::ERROR;
      }
    }

    // If any new command modes
    if (start_command_modes.size() != 0)
    {
      // Check they are all the same
      if (!std::equal(start_command_modes.begin() + 1, start_command_modes.end(), start_command_modes.begin()))
      {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_NAME), "Rejecting command mode switch: all joints must be in the same mode");
        return hardware_interface::return_type::ERROR;
      }
      // Position and velocity can have unclaimed joints because they can be held constant. Current/Torque need active control.
      // Check all are claimed if Current/Torque
      if ((start_command_modes.front() == "current" || start_command_modes.front() == "torque")
          && start_command_modes.size() != open_joints_.size()
          ){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_NAME), "Rejecting command mode switch: all joints must be claimed if using current or torque mode");
        return hardware_interface::return_type::ERROR;
      }

    }

    if (start_position_direct_count == 0){
      // Not using position_direct -- use generic versions
      if (start_command_modes.size() == 0){
        // No command claims, either continue previous mode or switch to listen as per stop_interface check
      }
      else if (start_command_modes.front() == hardware_interface::HW_IF_POSITION){
        next_control_mode = ControlMode::POSITION;
      }
      else if (start_command_modes.front() == hardware_interface::HW_IF_VELOCITY){
        next_control_mode = ControlMode::VELOCITY;
      }
      else if (start_command_modes.front() == hardware_interface::HW_IF_ACCELERATION){
        next_control_mode = ControlMode::ACCELERATION;
      }
      else if (start_command_modes.front() == "current"){
        next_control_mode = ControlMode::CURRENT;
      }
      else if (start_command_modes.front() == "torque"){
        next_control_mode = ControlMode::TORQUE;
      }
      else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_NAME), "Rejecting command mode switch: unrecognised command type, " << start_command_modes.front());
        return hardware_interface::return_type::ERROR;
      }
    } 
    else if (start_position_direct_count == open_joints_.size()){
      // Use position_direct versions
      if (start_command_modes.size() == 0){
        next_control_mode = ControlMode::POSITION_DIRECT;
      }
      else if (start_command_modes.front() == hardware_interface::HW_IF_POSITION){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_NAME), "Rejecting command mode switch: claim of both position and position_direct not allowed");
        return hardware_interface::return_type::ERROR;
      }
      else if (start_command_modes.front() == hardware_interface::HW_IF_VELOCITY){
        next_control_mode = ControlMode::VELOCITY_POSITION_DIRECT;
      }
      else if (start_command_modes.front() == "current"){
        next_control_mode = ControlMode::CURRENT_POSITION_DIRECT;
      }
      else if (start_command_modes.front() == "torque"){
        next_control_mode = ControlMode::TORQUE_POSITION_DIRECT;
      }
      else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_NAME), "Rejecting command mode switch: unrecognised command type with position_direct claims, " << start_command_modes.front());
        return hardware_interface::return_type::ERROR;
      }
    } 
    else {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_NAME), "Rejecting command mode switch: joints should either be all or none position_direct");
      return hardware_interface::return_type::ERROR;
    }

    if (next_control_mode != ControlMode::LISTEN && get_lifecycle_state().label() != hardware_interface::lifecycle_state_names::ACTIVE){
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Rejecting command mode switch: Hardware interface not ACTIVE so command mode switch out of LISTEN is not allowed.");
      return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type CRCOpenHardware::perform_command_mode_switch(
      const std::vector<std::string> & /*start_interfaces*/,
      const std::vector<std::string> & /*stop_interfaces*/)
  {
    RCLCPP_INFO(rclcpp::get_logger(LOG_NAME), "perform_command_mode_switch");

    if (next_control_mode != ControlMode::LISTEN && get_lifecycle_state().label() != hardware_interface::lifecycle_state_names::ACTIVE){
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Hardware interface not ACTIVE so command mode switch out of LISTEN is not allowed.");
      return hardware_interface::return_type::ERROR;
    }

    int err_status = ORL_ERR;

    // Set all axes to LISTEN first
    err_status = ORLD_SrvSetModality(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, open_axes_mask, ORL_LISTEN, false /*en_addon_speed*/, false /*en_addon_curr*/);
    if (err_status != ORL_OK)
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Failed command_mode_switch. Could not set modes to LISTEN: %d", err_status);
      return hardware_interface::return_type::ERROR;
    }

    // Read state so initial commands can be sensible
    read(rclcpp::Time{}, rclcpp::Duration(0, 0));

    // Set commands to retain status-quo by default
    for (Joint* joint: open_joints_)
    {
      uint axis_idx = joint->axis_idx;
      joint->command.position = joint->state.position;
      joint->command.velocity = 0;
      joint->command.acceleration = 0;
      joint->command.current = joint->state.current;
      joint->command.torque = joint->state.torque;

      // Set ruckig starting point
      ruckig_input.current_position[axis_idx] = joint->state.position;
      ruckig_input.current_velocity[axis_idx] = joint->state.velocity;
      ruckig_output.new_position[axis_idx] = joint->state.position;
      ruckig_output.new_velocity[axis_idx] = joint->state.velocity;

      // Reset max acceleration if changed by ACCELERATION mode
      ruckig_input.max_acceleration[axis_idx] = true_max_acceleration[axis_idx];
      // ruckig_input.min_acceleration.value()[axis_idx] = -true_max_acceleration[axis_idx];
    }

    // Set new modality
    switch (next_control_mode){
      case ControlMode::LISTEN:
        // Already in LISTEN
        break;
      case ControlMode::POSITION:
      case ControlMode::POSITION_DIRECT:
        ruckig_input.control_interface = ruckig::ControlInterface::Position;
        ruckig_input.target_position = ruckig_input.current_position;
        err_status = ORLD_SrvSetModality(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, open_axes_mask, ORL_POS_ABSOLUTE, false /*en_addon_speed*/, false /*en_addon_curr*/);
        break;
      case ControlMode::VELOCITY:
      case ControlMode::VELOCITY_POSITION_DIRECT:
      case ControlMode::ACCELERATION:
        ruckig_input.control_interface = ruckig::ControlInterface::Velocity;
        ruckig_input.target_velocity = ruckig_input.current_velocity;
        err_status = ORLD_SrvSetModality(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, open_axes_mask, ORL_SPD_FULL, false /*en_addon_speed*/, false /*en_addon_curr*/);
        break;
      
      case ControlMode::CURRENT:
      case ControlMode::CURRENT_POSITION_DIRECT:
      case ControlMode::TORQUE:
      case ControlMode::TORQUE_POSITION_DIRECT:
        err_status = ORLD_SrvSetModality(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, open_axes_mask, ORL_CURR_FULL, false /*en_addon_speed*/, false /*en_addon_curr*/);
        break;
    }
    if (err_status != ORL_OK)
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOG_NAME), "Failed command_mode_switch. ORLD_SrvSetModality Error: %d", err_status);
      return hardware_interface::return_type::ERROR;
    }

    // Update for next cycle
    control_mode = next_control_mode;
    next_control_mode = ControlMode::LISTEN;

    int modality;
    ORLD_GetModeAx(ORL_VERB_OFF,ORL_CNTRL1,ORL_AXIS1,open_joints_[0]->axis_idx,&modality); //Get mode of first open axis
    RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_NAME), "Control mode switched. CRC Control Mode: " << modality << ". control_mode enum: " << static_cast<int>(control_mode));

    return hardware_interface::return_type::OK;
  }

  hardware_interface::CallbackReturn CRCOpenHardware::on_error(const rclcpp_lifecycle::State & /* previous_state */)
  {
    // copy of close_app() of console.c

    int ModeMasterAx;
    ORLD_GetModeMasterAx(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, &ModeMasterAx);
    if (ModeMasterAx != ORL_LISTEN)
    {
      ORLD_SrvSetModality(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, open_axes_mask, ORL_LISTEN, false /*en_addon_speed*/, false /*en_addon_curr*/);
    }

    while (ModeMasterAx != ORL_LISTEN)
    {
      ORLD_GetModeMasterAx(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, &ModeMasterAx);
      rclcpp::sleep_for(std::chrono::seconds(1));
    }

    int StatusArm;
    ORLD_GetStatusArm(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, &StatusArm);
    if (StatusArm == ORL_DRIVEON)
      ORLD_SrvSetDrive(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, ORL_DRIVEOFF);

    while (StatusArm != ORL_DRIVEOFF)
    {
      ORLD_GetStatusArm(ORL_VERB_OFF, ORL_CNTRL1, ORL_ARM1, &StatusArm);
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
    ORLD_Close(ORL_VERB_OFF, ORL_CNTRL1);
    return hardware_interface::CallbackReturn::FAILURE; // could do SUCCESS to return to unconfigured
  };

  // CRCOpen Conversion Functions : See "02 - Architecture of Comau Open Controller.pdf"

  float CRCOpenHardware::crc_to_rad_pos(float crc_msg_pos, Joint joint){
    // Position chain computation from crc open message (motor rounds) to link (radians)
    float motor = crc_msg_pos - joint.cal_data; // Calibration constants
    float actuation = motor / joint.tx_rate; // Transmission rate
    // Note: Coupling effect add_influence() is zero for NJ130_2.6
    float link = actuation * (2*M_PI); // Position radians
    return link;
  }
  
  float CRCOpenHardware::rad_to_crc_pos(float link_rad_pos, Joint joint){
    // Position chain computation from link (radians) to crc open message (motor rounds)
    float actuation = link_rad_pos / (2*M_PI);
    // Note: Coupling effect remove_influence() is zero for NJ130_2.6
    float motor = actuation * joint.tx_rate; // Transmission rate
    float crc_msg = motor + joint.cal_data; // Calibration constants
    return crc_msg;
  }
  
  float CRCOpenHardware::crc_to_rad_vel(float crc_msg_vel, Joint joint){
    // Speed chain computation from crc open message (motor rounds / 400us) to link (radians / sec)
    float motor = crc_msg_vel / 0.0004; // convert to per sec
    float actuation = motor / joint.tx_rate; // Transmission rate
    // Note: Coupling effect add_influence() is zero for NJ130_2.6
    float link = actuation * (2*M_PI); // Position radians / sec
    return link;
  }
  
  float CRCOpenHardware::rad_to_crc_vel(float link_rad_vel, Joint joint){
    // Speed chain computation from link (radians / sec) to crc open message (motor rounds / 400us)
    float actuation = link_rad_vel / (2*M_PI);
    // Note: Coupling effect remove_influence() is zero for NJ130_2.6
    float motor = actuation * joint.tx_rate; // Transmission rate
    float crc_msg = motor * 0.0004; // convert to per 400us
    return crc_msg;
  }

  float CRCOpenHardware::trq_to_cur(float torque_Nm, Joint joint){
    // Convert from torque (Nm) to current (A)
    return torque_Nm / joint.vr_TorqConst;
  }
 
  float CRCOpenHardware::cur_to_trq(float current_A, Joint joint){
     // Convert from current (A) to torque (Nm)
    return current_A * joint.vr_TorqConst;
  }

} // namespace crcopen_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(crcopen_hardware::CRCOpenHardware, hardware_interface::SystemInterface)
