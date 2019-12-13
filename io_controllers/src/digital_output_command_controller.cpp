#include <algorithm>
#include <cstddef>

#include "io_controllers/digital_output_command_controller.h"
#include <pluginlib/class_list_macros.h>

namespace io_controllers
{
bool DigitalOutputCommandController::init(hardware_state_command_interfaces::DigitalOutputCommandInterface* hw,
                                          ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // Cache controller node handle
  controller_nh_ = controller_nh;
  // Controller name
  name_ = getLeafNamespace(controller_nh_);

  if (!controller_nh.getParam("publish_rate", publish_rate_))
  {
    ROS_ERROR("Parameter 'publish_rate' not set");
    return false;
  }

  if (!controller_nh_.getParam("digital_outputs", digital_output_names_))
  {
    ROS_ERROR("Parameter 'digital_outputs' not set");
    return false;
  }

  //    // State publish rate
  //    double state_publish_rate = 50.0;
  //    controller_nh_.getParam("state_publish_rate", state_publish_rate);
  //    ROS_DEBUG_STREAM_NAMED(name_, "Controller state will be published at " << state_publish_rate << "Hz.");
  //    state_publisher_period_ = ros::Duration(1.0 / state_publish_rate);

  // TODO: get start state from the YAML file
  // get DigitalOutput name from the parameter server
  //  std::vector<std::string> digital_output_names;

  //  if (!controller_nh_.getParam("digital_outputs", digital_output_names))
  //  {
  //    ROS_ERROR("Could not find digital output name");
  //    return false;
  //  }
  num_hw_outputs_ = digital_output_names_.size();
  for (unsigned i = 0; i < num_hw_outputs_; i++)
    ROS_DEBUG("Got DO %s", digital_output_names_[i].c_str());

  //  // get publishing period
  //  if (!controller_nh.getParam("publish_rate", publish_rate_))
  //  {
  //    ROS_ERROR("Parameter 'publish_rate' not set");
  //    return false;
  //  }

  // realtime publisher
  // realtime_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(root_nh, "joint_states", 4));

  // ROS API: Subscribed topics
  output_command_sub_ = controller_nh_.subscribe("command", 1, &DigitalOutputCommandController::outputCommandCB, this);

  // get joints and allocate message
  for (unsigned i = 0; i < num_hw_outputs_; i++)
  {
    ROS_INFO_STREAM("registered to digital output handle: " << digital_output_names_[i]);
    digital_output_command_.push_back(hw->getHandle(digital_output_names_[i]));
    //      realtime_pub_->msg_.name.push_back(digital_output_names[i]);
    //      realtime_pub_->msg_.position.push_back(0.0);
    //      realtime_pub_->msg_.velocity.push_back(0.0);
    //      realtime_pub_->msg_.effort.push_back(0.0);
  }

  return true;
}

void DigitalOutputCommandController::starting(const ros::Time& time)
{
  // initialize time
  last_publish_time_ = time;
  CommandPtr command_ptr(new Command);
  *command_ptr = hardware_state_command_interfaces::DigitalIOStateHandle::State::UNDEFINED;
  current_command_box_.set(command_ptr);
}

void DigitalOutputCommandController::update(const ros::Time& time, const ros::Duration& /*period*/)
{
  CommandPtr current_command_ptr;
  current_command_box_.get(current_command_ptr);
  Command& current_command = *current_command_ptr;
  digital_output_command_[0].setCommand(current_command);
  //  // limit rate of publishing
  //  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
  //  {
  //    // try to publish
  //    if (realtime_pub_->trylock())
  //    {
  //      // we're actually publishing, so increment time
  //      last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

  //      // populate joint state message:
  //      // - fill only joints that are present in the JointStateInterface, i.e. indices [0, num_hw_joints_)
  //      // - leave unchanged extra joints, which have static values, i.e. indices from num_hw_joints_ onwards
  //      realtime_pub_->msg_.header.stamp = time;
  //      for (unsigned i = 0; i < num_hw_joints_; i++)
  //      {
  //        realtime_pub_->msg_.position[i] = joint_state_[i].getPosition();
  //        realtime_pub_->msg_.velocity[i] = joint_state_[i].getVelocity();
  //        realtime_pub_->msg_.effort[i] = joint_state_[i].getEffort();
  //      }
  //      realtime_pub_->unlockAndPublish();
  //    }
  //  }
}

void DigitalOutputCommandController::stopping(const ros::Time& /*time*/)
{
}

std::string DigitalOutputCommandController::getLeafNamespace(const ros::NodeHandle& nh)
{
  const std::string complete_ns = nh.getNamespace();
  std::size_t id = complete_ns.find_last_of("/");
  return complete_ns.substr(id + 1);
}
void DigitalOutputCommandController::outputCommandCB(const std_msgs::Bool& msg)
{
  CommandPtr current_command_ptr;
  current_command_box_.get(current_command_ptr);
  // Update currently executing command
  try
  {
    CommandPtr command_ptr(new Command);
    *command_ptr = msg.data ? Command::HIGH : Command::LOW;
    current_command_box_.set(command_ptr);
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM_NAMED(name_, ex.what());
  }
}

}  // namespace io_controllers

PLUGINLIB_EXPORT_CLASS(io_controllers::DigitalOutputCommandController, controller_interface::ControllerBase)
