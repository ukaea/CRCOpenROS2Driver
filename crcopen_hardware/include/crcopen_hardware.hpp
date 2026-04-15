#ifndef CRCOPEN_HARDWARE__CRCOPEN_HARDWARE_HPP_
#define CRCOPEN_HARDWARE__CRCOPEN_HARDWARE_HPP_

// Comau CRCOpen Libraries
#include <orl_driver.h>
#include <orl_util.h>

// Ruckig Online Trajectory Generation
#include <ruckig/ruckig.hpp>

// C++ Standard Libraries
#include <string>
#include <vector>
#include <map>
#include <thread>

// ROS2 Control Libraries
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace crcopen_hardware
{

enum class ControlMode {
  LISTEN,
  POSITION,
  POSITION_DIRECT,
  VELOCITY,
  VELOCITY_POSITION_DIRECT,
  ACCELERATION,
  CURRENT,
  CURRENT_POSITION_DIRECT,
  TORQUE,
  TORQUE_POSITION_DIRECT,
};

struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double acceleration{0.0};
  double current{0.0};
  double torque{0.0};
};

struct Joint
{
  std::string name;
  unsigned int axis_idx;

  JointValue state{};
  JointValue command{};

  double cal_data;
  double tx_rate;
  double vr_TorqConst;
  double vr_TransmissionRatio;
};

/**
 * @brief Hardware interface for Comau CRC Open (C5G) using the ORL driver.
 *
 * Implements ros2_control SystemInterface, performs unit conversion and optional
 * Ruckig interpolation, and exchanges data with the CRCOpen controller via ORL.
 * 
 * @note The methods prefixed with on_*, along with export_*, read, write,
 *       and the command-mode switch functions, are the standard ros2_control
 *       lifecycle hooks (not custom entry points). See:
 *       https://control.ros.org/rolling/doc/resources/resources.html
 */
class CRCOpenHardware
: public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CRCOpenHardware)

   /**
   * @brief Initialize hardware from URDF/ros2_control description.
   *
   * Reads hardware parameters from info, sets up internal buffers and Ruckig.
   * No network communication is started.
   *
   * @param[in] info Hardware description from the ResourceManager.
   * @return SUCCESS if initialized; FAILURE if parameters are missing/invalid; ERROR otherwise.
   * @post UNCONFIGURED state on success.
   */
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;

  /**
   * @brief Export per-joint state interfaces.
   *
   * Exposes state interfaces for position, velocity, acceleration, current, and effort.
   *
   * @return Vector of StateInterface objects.
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    /**
   * @brief Export per-joint command interfaces.
   *
   * Exposes command interfaces: position, position_direct, velocity, acceleration, current, effort.
   *
   * @return Vector of CommandInterface objects.
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief Establish ORL communication and discover open axes.
   *
   * Opens the CRC connection, starts the IO thread, sets payload, and builds the list of open joints.
   *
   * @param[in] previous_state Lifecycle state prior to configuration.
   * @return SUCCESS on configuration; FAILURE for recoverable issues; ERROR otherwise.
   * @post INACTIVE state on success.
   */
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Enable motion (DRIVES ON).
   *
   * Requires AUTO TP mode, requests DRIVES ON, and waits for confirmation.
   *
   * @param[in] previous_state Lifecycle state prior to activation.
   * @return SUCCESS when active; FAILURE if preconditions not met; ERROR on communication errors.
   * @post ACTIVE state on success.
   */
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Stop motion (LISTEN + DRIVES OFF).
   *
   * Switches to LISTEN and turns drives off.
   *
   * @param[in] previous_state Lifecycle state prior to deactivation.
   * @return SUCCESS on deactivation; ERROR otherwise.
   * @post INACTIVE state on success.
   */
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Close ORL communication and stop IO thread.
   *
   * @param[in] previous_state Lifecycle state prior to cleanup.
   * @return SUCCESS on cleanup; ERROR otherwise.
   * @post UNCONFIGURED state on success.
   */
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Graceful shutdown from any state.
   *
   * Calls deactivate and cleanup.
   *
   * @param[in] previous_state Lifecycle state prior to shutdown.
   * @return SUCCESS on shutdown; ERROR otherwise.
   * @post FINALIZED state on success.
   */
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Read latest measurements into state interfaces.
   *
   * Copies buffered IO data, converts units, and updates state interfaces.
   *
   * @param[in] time   Current time (unused).
   * @param[in] period Time since last read (unused).
   * @return OK on success; ERROR on failure.
   */
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  /**
   * @brief Write current commands towards the hardware.
   *
   * Updates Ruckig targets or caches direct commands; the IO thread performs the ORL writes.
   *
   * @param[in] time   Current time (unused).
   * @param[in] period Time since last write (unused).
   * @return OK on success; ERROR on failure.
   */
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief Check and prepare a command-mode switch request.
   *
   * Validates requested interfaces and computes the next control mode.
   *
   * @param[in] start_interfaces Fully-qualified interfaces to start (e.g., "joint/velocity").
   * @param[in] stop_interfaces  Fully-qualified interfaces to stop (unused).
   * @return OK if acceptable; ERROR if invalid.
   */
  hardware_interface::return_type prepare_command_mode_switch(
                                      const std::vector<std::string> & start_interfaces,
                                      const std::vector<std::string> & /*stop_interfaces*/) override;

  /**
   * @brief Apply a previously prepared command-mode switch.
   *
   * Sets CRC modality via ORL, resets Ruckig state, and updates control_mode.
   *
   * @param[in] start_interfaces Interfaces to start (unused).
   * @param[in] stop_interfaces  Interfaces to stop (unused).
   * @return OK on success; ERROR otherwise.
   */
  hardware_interface::return_type perform_command_mode_switch(
                                      const std::vector<std::string> & start_interfaces,
                                      const std::vector<std::string> & /*stop_interfaces*/) override;

  /**
   * @brief Handle errors raised from any lifecycle state.
   *
   * Attempts to switch to LISTEN, turn drives off, and close ORL.
   *
   * @param[in] previous_state Lifecycle state prior to the error.
   * @return FAILURE (component transitions to FINALIZED).
   */
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

protected:
  /**
   * Threaded function that maintains CRC communication.
   * 
   * This is a critical real-time function that uses the ORL Driver functions to
   * read and write to the robot motors. The Ruckig interpolator is called every
   * loop to smooth the trajectory where needed.
   */
  void orld_thread_function();

  std::thread orld_thread_handle;
  bool keep_thread_running;

  // CRCOpen Conversion Calculations
  /**
   * Convert joint position from CRC Message to Radians
   * @param crc_msg_pos Position (motor rounds)
   * @param joint Joint struct with calibration constants
   * @return Position (radians)
   */
  float crc_to_rad_pos(float crc_msg_pos, Joint joint);
  /**
   * Convert joint position from Radians to CRC Message
   * @param link_rad_pos Position (radians)
   * @param joint Joint struct with calibration constants
   * @return Position (motor rounds)
   */
  float rad_to_crc_pos(float link_rad_pos, Joint joint);
  /**
   * Convert joint velocity from CRC Message to Radians per Second
   * @param crc_msg_vel Velocity (motor rounds / time step)
   * @param joint Joint struct with calibration constants
   * @return Velocity (radians / second)
   */
  float crc_to_rad_vel(float crc_msg_vel, Joint joint);
  /**
   * Convert joint velocity from Radians per Second to CRC Message
   * @param crc_msg_vel Velocity (radians / second)
   * @param joint Joint struct with calibration constants
   * @return Velocity (motor rounds / time step)
   */
  float rad_to_crc_vel(float link_rad_vel, Joint joint);
  /**
   * Calculate joint torque from motor current
   * @param torque_Nm Torque (Newton-Meters)
   * @param joint Joint struct with calibration constants
   * @return Current (amperes)
   */
  float trq_to_cur(float torque_Nm, Joint joint);
  /**
   * Calculate motor current from joint torque
   * @param current_A Current (amperes)
   * @param joint Joint struct with calibration constants
   * @return Torque (Newton-Meters)
   */
  float cur_to_trq(float current_A, Joint joint);

  // read/write values for ros2control command/state
  std::vector<Joint> joints_; //In whatever order ros2control gives
  std::vector<Joint*> open_joints_; //Pointers to elements in joints_ corresponding to the joints in open mode
  double callback_period_;

  ControlMode control_mode; // All axes have to be same, so control mode is shared
  ControlMode next_control_mode; // For command mode switching

  // Network addresses needed for controller intialisations
  const char* LPC_addr;
  const char* CRC_addr;

  // Payload Parameters to be passed to C5G
  float payload_mass; //Kg
  float payload_cog[3]; //mm
  float payload_inertia[6]; //Kg*m^2

  // Data structures for ruckig Online Trajectory Generation (OTG). Used for interpolation in position mode
  ruckig::Ruckig<6> ruckig_otg;
  ruckig::InputParameter<6> ruckig_input;
  ruckig::OutputParameter<6> ruckig_output;
  double true_max_acceleration[6]; // Store used because ACCELERATION control mode overrides ruckig_input.max_acceleration

  // read/write values for ORLD cycle (strictly ordered by axis index)
  float orld_pos_meas[6];
  float orld_vel_meas[6];
  float orld_cur_meas[6];

  float orld_pos_target[6];
  float orld_vel_target[6];
  float orld_cur_target[6];
  float orld_trq_target[6];

  // Used for ORLD_SrvSetModality. Set in on_configure()
  long open_axes_mask = 0b000000;

};
}  // namespace crcopen_hardware

#endif  // CRCOPEN_HARDWARE__CRCOPEN_HARDWARE_HPP_
