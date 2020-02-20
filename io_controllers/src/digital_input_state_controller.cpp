#include "io_controllers/digital_input_state_controller.h"

#include <pluginlib/class_list_macros.h>

namespace io_controllers
{
bool DigitalInputStateController::init(hardware_state_command_interfaces::DigitalInputStateInterface* hw,
                                       ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)

{
  // Cache controller node handle
  controller_nh_ = controller_nh;

  // Controller name
  name_ = getLeafNamespace(controller_nh_);

  // get publishing period
  if (!controller_nh_.getParam("publish_rate", publish_rate_))
  {
    ROS_ERROR("Parameter 'publish_rate' not set");
    return false;
  }

  if (!controller_nh_.getParam("digital_inputs", digital_input_names_))
  {
    ROS_ERROR("Parameter 'digital_inputs' not set");
    return false;
  }

  if (controller_nh_.getParam("is_composite", is_inside_composite_controller))
  {
    ROS_INFO("Controller is composte controller");
  }

  num_hw_inputs_ = digital_input_names_.size();
  for (unsigned i = 0; i < num_hw_inputs_; i++)
  {
    ROS_INFO("Got DI %s", digital_input_names_[i].c_str());
  }

  // realtime publisher
  std::string state_topic = "state";
  if (is_inside_composite_controller)
  {
    state_topic = "digital_inputs/state";
  }
  realtime_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::ByteMultiArray>(controller_nh_, state_topic, 1));

  // get DIs and allocate message
  for (unsigned i = 0; i < num_hw_inputs_; i++)
  {
    ROS_INFO_STREAM("registered to digital input handle: " << digital_input_names_[i]);
    if (digital_input_names_[i] == "STATE.LOW")
    {
      hardware_state_command_interfaces::DigitalIOStateHandle::State state =
          hardware_state_command_interfaces::DigitalIOStateHandle::State::LOW;
      digital_input_state_.push_back(
          hardware_state_command_interfaces::DigitalIOStateHandle(digital_input_names_[i], &state));
    }
    else if (digital_input_names_[i] == "STATE.HIGH")
    {
      hardware_state_command_interfaces::DigitalIOStateHandle::State state =
          hardware_state_command_interfaces::DigitalIOStateHandle::State::HIGH;
      digital_input_state_.push_back(
          hardware_state_command_interfaces::DigitalIOStateHandle(digital_input_names_[i], &state));
    }
    else
    {
      digital_input_state_.push_back(hw->getHandle(digital_input_names_[i]));
    }
  }

  {
    realtime_pub_->lock();
    realtime_pub_->msg_.data.resize(num_hw_inputs_);
    realtime_pub_->unlock();
  }

  return true;
}

void DigitalInputStateController::starting(const ros::Time& time)
{
  if (realtime_pub_->trylock())
  {
    // initialize time
    last_publish_time_ = time;

    // TODO::define custome type to heave time stamp wrealtime_pub_->msg_.header.stamp = time;
    for (unsigned i = 0; i < num_hw_inputs_; i++)
    {
      realtime_pub_->msg_.data[i] =
          digital_input_state_[i].getState() == hardware_state_command_interfaces::DigitalIOStateHandle::State::HIGH;
    }
    realtime_pub_->unlockAndPublish();
  }
}

void DigitalInputStateController::update(const ros::Time& time, const ros::Duration& /*period*/)
{
  // limit rate of publishing
  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
  {
    // try to publish
    if (realtime_pub_->trylock())
    {
      // we're actually publishing, so increment time
      last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

      // TODO::define custome type to heave time stamp wrealtime_pub_->msg_.header.stamp = time;
      for (unsigned i = 0; i < num_hw_inputs_; i++)
      {
        realtime_pub_->msg_.data[i] =
            digital_input_state_[i].getState() == hardware_state_command_interfaces::DigitalIOStateHandle::State::HIGH;
      }
      realtime_pub_->unlockAndPublish();
    }
  }
}

void DigitalInputStateController::stopping(const ros::Time& /*time*/)
{
}

std::string DigitalInputStateController::getLeafNamespace(const ros::NodeHandle& nh)
{
  const std::string complete_ns = nh.getNamespace();
  std::size_t id = complete_ns.find_last_of("/");
  return complete_ns.substr(id + 1);
}

}  // namespace io_controllers

PLUGINLIB_EXPORT_CLASS(io_controllers::DigitalInputStateController, controller_interface::ControllerBase)
