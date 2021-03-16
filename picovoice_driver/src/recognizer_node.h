#pragma once

#include <actionlib/server/simple_action_server.h>
#include <ros/node_handle.h>
#include <ros/package.h>
#include <string>
#include <thread>

#include "./util.h"

namespace picovoice_driver
{
std::string defaultResourcePath()
{
  auto pkg_path = ros::package::getPath("picovoice_driver");
  if (pkg_path.empty())
  {
    throw std::runtime_error("Could not find picovoice_driver package");
  }
  return pkg_path + "/extern/picovoice/resources";
}

template <typename RecognizerDataType, typename RecognizerType, typename ActionType>
class RecognizerNode
{
public:
  explicit RecognizerNode(const std::string& name, const std::string& default_model_path)
    : server_(name, boost::bind(&RecognizerNode::executeCallback, this, _1), false)
  {
    ros::NodeHandle local_nh("~");
    parameters_.model_path_ = local_nh.param("model_path", default_model_path);
    parameters_.sensitivity_ = local_nh.param("sensitivity", parameters_.sensitivity_);
    server_.start();

    ROS_INFO("RecognizerNode(%s) initialized", name.c_str());
  }

private:
  typename RecognizerDataType::Parameters parameters_;
  RecognizerType recognizer_;

  virtual void updateParameters(const typename ActionType::_action_goal_type::_goal_type& goal,
                                typename RecognizerDataType::Parameters& parameters) = 0;
  virtual void updateResult(const typename RecognizerDataType::Result& result,
                            typename ActionType::_action_result_type::_result_type& action_result) = 0;

  std::string recognize_thread_error_;
  void recognizeThread()
  {
    ROS_INFO("Recognizing ..");
    recognize_thread_error_.clear();
    try
    {
      recognizer_.recognize();
    }
    catch (const std::exception& e)
    {
      recognize_thread_error_ = std::string(e.what());
    }
    ROS_INFO("Recognizing done");
  }

  actionlib::SimpleActionServer<ActionType> server_;
  void executeCallback(const typename ActionType::_action_goal_type::_goal_type::ConstPtr& goal)
  {
    typename ActionType::_action_result_type::_result_type action_result;

    try
    {
      updateParameters(*goal, parameters_);
    }
    catch (const std::exception& e)
    {
      std::string msg = "Invalid goal: " + std::string(e.what());
      ROS_ERROR("%s", msg.c_str());
      server_.setAborted(action_result, msg);
      return;
    }

    try
    {
      recognizer_.configure(parameters_);
      ROS_INFO("Configured recognizer: %s", toString(parameters_).c_str());
    }
    catch (const std::exception& e)
    {
      std::string msg = "Could not configure recognizer: " + std::string(e.what());
      ROS_ERROR("%s", msg.c_str());
      server_.setAborted(action_result, msg);
      return;
    }

    std::thread t(&RecognizerNode::recognizeThread, this);
    while (ros::ok() && recognizer_.isRecognizing())
    {
      if (server_.isPreemptRequested())
      {
        recognizer_.preempt();
      }
    }
    t.join();

    if (!recognize_thread_error_.empty())
    {
      std::string msg = "Recognize error: " + recognize_thread_error_;
      ROS_ERROR("%s", msg.c_str());
      server_.setAborted(action_result, msg);
      return;
    }

    auto result = recognizer_.getResult();
    ROS_INFO("Result: %s", toString(result).c_str());

    updateResult(result, action_result);
    if (server_.isPreemptRequested())
    {
      server_.setPreempted(action_result);
    }
    else
    {
      server_.setSucceeded(action_result);
    }
  }
};
}  // namespace picovoice_driver
