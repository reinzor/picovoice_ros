#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/package.h>

#include "./rhino_recognizer.h"
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
  ros::init(argc, argv, "rhino");

  ros::NodeHandle local_nh("~");
  auto default_resource_path = defaultResourcePath();
  RhinoRecognizerData::Parameters parameters;
  parameters.model_path_ = local_nh.param("model_path", default_resource_path + "/models/rhino_params.pv");
  parameters.context_path_ = local_nh.param("context_path", default_resource_path + "/contexts/coffee_maker_linux.rhn");
  parameters.sensitivity_ = local_nh.param("sensitivity", parameters.sensitivity_);

  try
  {
    RhinoRecognizer recognizer;
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
