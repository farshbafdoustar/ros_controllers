

#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/SwitchController.h>
class DigitalOutputControllerTest : public ::testing::Test
{
public:
  DigitalOutputControllerTest()
  {
  }
  ~DigitalOutputControllerTest()
  {
    // state_sub.shutdown(); // This is important, to make sure that the callback is not woken up later in the
    // destructor
  }

protected:
  ros::NodeHandle nh;
  ros::Subscriber state_sub;
};

TEST_F(DigitalOutputControllerTest, stateTopicConsistency)
{
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "digital_output_command_controller_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
