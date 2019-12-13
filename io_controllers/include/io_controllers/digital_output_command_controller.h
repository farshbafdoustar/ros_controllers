#ifndef IO_CONTROLLERS_DIGITAL_OUTPUT_COMMAND_CONTROLLER
#define IO_CONTROLLERS_DIGITAL_OUTPUT_COMMAND_CONTROLLER

// C++ standard
#include <cassert>
#include <stdexcept>
#include <string>
#include <memory>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/dynamic_bitset.hpp>

// ROS
#include <ros/node_handle.h>
#include <std_msgs/Bool.h>

// realtime_tools
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// ros_controls
#include <controller_interface/controller.h>

#include <hardware_state_command_interfaces/digital_io_command_interface.h>

namespace io_controllers
{
class DigitalOutputCommandController
  : public controller_interface::Controller<hardware_state_command_interfaces::DigitalOutputCommandInterface>
{
public:
  DigitalOutputCommandController() : publish_rate_(0.0)
  {
  }
  virtual bool init(hardware_state_command_interfaces::DigitalOutputCommandInterface* hw, ros::NodeHandle& root_nh,
                    ros::NodeHandle& controller_nh);

  virtual void update(const ros::Time& time, const ros::Duration& period);

  virtual void starting(const ros::Time& time);

  virtual void stopping(const ros::Time& time);

protected:
  std::string getLeafNamespace(const ros::NodeHandle& nh);

  void outputCommandCB(const std_msgs::Bool& msg);

  typedef hardware_state_command_interfaces::DigitalIOStateHandle::State Command;
  typedef std::shared_ptr<hardware_state_command_interfaces::DigitalIOStateHandle::State> CommandPtr;
  typedef realtime_tools::RealtimeBox<CommandPtr> CommandBox;

  /**
   * Thread-safe container with a smart pointer to command currently being followed.
   * Can be either a hold command or a command received from a ROS message.
   *
   * We store the hold command in a separate class member because the \p starting(time) method must be realtime-safe.
   * The (single segment) hold command is preallocated at initialization time and its size is kept unchanged.
   */
  CommandBox current_command_box_;
  CommandPtr hold_command_ptr_;  ///< Last hold state values.

  // TODO::Array definition
  std::vector<hardware_state_command_interfaces::DigitalOutputHandle> digital_output_command_;
  // hardware_state_command_interfaces::DigitalIOStateHandle digital_output_command_;

  // ROS API
  ros::NodeHandle controller_nh_;
  ros::Subscriber output_command_sub_;
  // std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > realtime_pub_;
  ros::Time last_publish_time_;
  double publish_rate_;
  unsigned int num_hw_outputs_;
  std::string name_;  ///< Controller name.
  std::vector<std::string> digital_output_names_;

  bool is_inside_composite_controller = false;
};

}  // namespace io_controllers

#endif  // IO_CONTROLLERS_DIGITAL_OUTPUT_COMMAND_CONTROLLER
