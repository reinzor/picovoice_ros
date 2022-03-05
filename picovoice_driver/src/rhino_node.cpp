/*
 * Copyright 2021, Rein Appeldoorn
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

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
  RhinoNode(const RhinoRecognizerData::Parameters& parameters, const std::string& contexts_directory_url)
    : RecognizerNode("rhino", "get_intent", parameters), contexts_directory_url_(contexts_directory_url)
  {
  }

private:
  void updateParameters(const GetIntentGoal& goal, RhinoRecognizerData::Parameters& parameters) override
  {
    parameters.context_path_ = pathFromUrl(goal.context_url, ".rhn", contexts_directory_url_);
    parameters.require_endpoint_ = goal.require_endpoint;
    parameters.intents_ = goal.intents;
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

  std::string contexts_directory_url_;
};
}  // namespace picovoice_driver

int main(int argc, char** argv)
{
  using namespace picovoice_driver;

  ros::init(argc, argv, "rhino");

  ros::NodeHandle local_nh("~");
  std::string model_url = local_nh.param("model_url", defaultResourceUrl() + "/models/rhino_params.pv");
  std::string contexts_directory_url = local_nh.param("contexts_directory_url", defaultResourceUrl() + "/contexts");

  try
  {
    RhinoRecognizerData::Parameters parameters;
    parameters.model_path_ = pathFromUrl(model_url);

    RhinoNode node(parameters, pathFromUrl(contexts_directory_url));
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL("RhinoNode exception: %s", e.what());
    return 1;
  }
  return 0;
}
