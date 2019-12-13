#ifndef IO_CONTROLLERS_DIGITAL_INPUT_STATE_CONTROLLER
#define IO_CONTROLLERS_DIGITAL_INPUT_STATE_CONTROLLER

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
#include <std_msgs/ByteMultiArray.h>

// realtime_tools
#include <realtime_tools/realtime_publisher.h>

// ros_controls
#include <controller_interface/controller.h>

#include <hardware_state_command_interfaces/digital_io_command_interface.h>

namespace io_controllers
{
class DigitalInputStateController
  : public controller_interface::Controller<hardware_state_command_interfaces::DigitalInputStateInterface>
{
public:
  DigitalInputStateController() : publish_rate_(0.0)
  {
  }
  virtual bool init(hardware_state_command_interfaces::DigitalInputStateInterface* hw, ros::NodeHandle& root_nh,
                    ros::NodeHandle& controller_nh);

  virtual void update(const ros::Time& time, const ros::Duration& period);

  virtual void starting(const ros::Time& time);

  virtual void stopping(const ros::Time& time);

protected:
  std::string getLeafNamespace(const ros::NodeHandle& nh);

  // TODO::add service to query state
  // void outputCommandCB(const std_msgs::Bool& msg);

  typedef hardware_state_command_interfaces::DigitalIOStateHandle::State State;
  typedef std::shared_ptr<hardware_state_command_interfaces::DigitalIOStateHandle::State> StatePtr;

  // TODO::Array definition
  std::vector<hardware_state_command_interfaces::DigitalIOStateHandle> digital_input_state_;
  // hardware_state_command_interfaces::DigitalIOStateHandle digital_output_command_;

  // ROS API
  ros::NodeHandle controller_nh_;
  // ros::Subscriber output_command_sub_;
  // TODO:: change publisher to publisher more than one DI
  // find better typ for emulti bool values
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::ByteMultiArray>> realtime_pub_;
  ros::Time last_publish_time_;
  double publish_rate_;
  unsigned int num_hw_inputs_;
  std::string name_;  ///< Controller name.
  std::vector<std::string> digital_input_names_;

  bool is_inside_composite_controller = false;
};

}  // namespace io_controllers

#endif  // IO_CONTROLLERS_DIGITAL_OUTPUT_COMMAND_CONTROLLER
