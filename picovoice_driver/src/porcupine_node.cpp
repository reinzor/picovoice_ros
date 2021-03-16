#include <picovoice_msgs/GetWakeWordAction.h>
#include <ros/init.h>

#include "./porcupine_recognizer.h"
#include "./recognizer_node.h"

namespace picovoice_driver
{
using namespace picovoice_msgs;
class PorcupineNode : public RecognizerNode<PorcupineRecognizerData, PorcupineRecognizer, GetWakeWordAction>
{
public:
  PorcupineNode() : RecognizerNode("get_wake_word", defaultResourcePath() + "/models/porcupine_params.pv")
  {
  }

private:
  void updateParameters(const GetWakeWordGoal& goal, PorcupineRecognizerData::Parameters& parameters) override
  {
    parameters.keyword_path_ = defaultResourcePath() + "/keyword_files/porcupine_linux.ppn";
  }

  void updateResult(const PorcupineRecognizerData::Result& result, GetWakeWordResult& action_result) override
  {
    action_result.is_understood = result.is_understood_;
    action_result.keyword = result.keyword_;
  }
};
}  // namespace picovoice_driver

int main(int argc, char** argv)
{
  ros::init(argc, argv, "porcupine");
  try
  {
    picovoice_driver::PorcupineNode node;
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL("PorcupineNode exception: %s", e.what());
    return 1;
  }
  return 0;
}
