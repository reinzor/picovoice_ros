#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/package.h>

#include "./porcupine_recognizer.h"
#include "./util.h"

using namespace picovoice_driver;

std::string defaultResourcePath()
{
  auto pkg_path = ros::package::getPath("picovoice_driver");
  if (pkg_path.empty())
  {
    throw std::runtime_error("Could not find picovoice_driver package");
  }
  return pkg_path + "/extern/picovoice/resources";
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "porcupine");

  ros::NodeHandle local_nh("~");
  auto default_resource_path = defaultResourcePath();
  PorcupineRecognizerData::Parameters parameters;
  parameters.model_path_ = local_nh.param("model_path", default_resource_path + "/models/porcupine_params.pv");
  parameters.keyword_path_ = local_nh.param("keyword_path", default_resource_path + "/keyword_files/"
                                                                                    "porcupine_linux.ppn");
  parameters.sensitivity_ = local_nh.param("sensitivity", parameters.sensitivity_);

  try
  {
    PorcupineRecognizer recognizer;
    recognizer.configure(parameters);
    recognizer.recognize();
    auto result = recognizer.getResult();

    ROS_INFO("Result: %s", toString(result).c_str());
  }
  catch (const std::exception& e)
  {
    ROS_FATAL("Recognizer exception: %s", e.what());
  }

  return 0;
}
