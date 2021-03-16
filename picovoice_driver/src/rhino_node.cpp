#include <picovoice_msgs/GetIntentAction.h>
#include <ros/init.h>

#include "./rhino_recognizer.h"
#include "./recognizer_node.h"

namespace picovoice_driver
{
using namespace picovoice_msgs;
class RhinoNode : public RecognizerNode<RhinoRecognizerData, RhinoRecognizer, GetIntentAction>
{
public:
  RhinoNode() : RecognizerNode("get_intent", defaultResourcePath() + "/models/rhino_params.pv")
  {
  }

private:
  void updateParameters(const GetIntentGoal& goal, RhinoRecognizerData::Parameters& parameters) override
  {
    parameters.context_path_ = defaultResourcePath() + "/contexts/coffee_maker_linux.rhn";
  }

  void updateResult(const RhinoRecognizerData::Result& result, GetIntentResult& action_result) override
  {
    action_result.is_understood = result.is_understood_;
    action_result.intent = result.intent_;
    for (const auto& slot : result.slots_)
    {
      picovoice_msgs::KeyValue kv;
      kv.key = slot.key_;
      kv.value = slot.value_;
      action_result.slots.push_back(kv);
    }
  }
};
}  // namespace picovoice_driver

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rhino");
  try
  {
    picovoice_driver::RhinoNode node;
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL("RhinoNode exception: %s", e.what());
    return 1;
  }
  return 0;
}
