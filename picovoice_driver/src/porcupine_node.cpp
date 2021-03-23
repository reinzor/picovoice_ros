#include <picovoice_msgs/GetKeywordAction.h>
#include <ros/init.h>

#include "./porcupine_recognizer.h"
#include "./recognizer_node.h"
#include "./ros_util.h"

namespace picovoice_driver
{
using namespace picovoice_msgs;
class PorcupineNode : public RecognizerNode<PorcupineRecognizerData, PorcupineRecognizer, GetKeywordAction>
{
public:
  PorcupineNode(const PorcupineRecognizerData::Parameters& parameters, const std::string& keywords_directory)
    : RecognizerNode("get_keyword", parameters), keywords_directory_(keywords_directory)
  {
  }

private:
  void updateParameters(const GetKeywordGoal& goal, PorcupineRecognizerData::Parameters& parameters) override
  {
    if (goal.keywords.empty())
    {
      throw std::runtime_error("No keywords specified");
    }

    parameters.keywords_.clear();
    for (const auto& keyword : goal.keywords)
    {
      parameters.keywords_[keyword.name] = pathFromUrl(keyword.url, ".ppn", keywords_directory_);
    }
  }

  void updateResult(const PorcupineRecognizerData::Result& result, GetKeywordResult& action_result) override
  {
    action_result.is_understood = result.is_understood_;
    action_result.keyword_name = result.keyword_name_;
  }

  std::string keywords_directory_;
};
}  // namespace picovoice_driver

int main(int argc, char** argv)
{
  using namespace picovoice_driver;

  ros::init(argc, argv, "porcupine");

  ros::NodeHandle local_nh("~");
  std::string model_url = local_nh.param("model_url", defaultResourceUrl() + "/models/porcupine_params.pv");
  std::string keywords_directory_url = local_nh.param("keywords_directory_url", defaultResourceUrl() + "/keywords");

  try
  {
    PorcupineRecognizerData::Parameters parameters;
    parameters.model_path_ = pathFromUrl(model_url, ".pv");

    PorcupineNode node(parameters, pathFromUrl(keywords_directory_url));
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL("PorcupineNode exception: %s", e.what());
    return 1;
  }
  return 0;
}
