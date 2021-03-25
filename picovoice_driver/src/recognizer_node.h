#pragma once

#include <actionlib/server/simple_action_server.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <ros/node_handle.h>
#include <string>

#include "./ros_util.h"
#include "./util.h"

namespace picovoice_driver
{
template <typename RecognizerDataType, typename RecognizerType, typename ActionType>
class RecognizerNode
{
public:
  explicit RecognizerNode(const std::string& name, const typename RecognizerDataType::Parameters& parameters)
    : action_server_(name, boost::bind(&RecognizerNode::executeCallback, this, _1), false), parameters_(parameters)
  {
    ros::NodeHandle pnh("~");
    pnh.param("execute_period", execute_period_, execute_period_);
    recognizer_.initialize(pnh.param("record_directory", defaultRecordDirectory()), pnh.param("max_record_length", 10));

    dynamic_reconfigure_server_.registerVariable<double>("sensitivity", &parameters_.sensitivity_,
                                                         "Recognizer sensitivity", 0., 1.);
    dynamic_reconfigure_server_.publishServicesTopics();

    action_server_.start();
    ROS_INFO("RecognizerNode(name=%s, execute_period=%.2f) initialized with parameters %s", name.c_str(),
             execute_period_, toString(parameters_).c_str());
  }

private:
  typename RecognizerDataType::Parameters parameters_;
  ddynamic_reconfigure::DDynamicReconfigure dynamic_reconfigure_server_;

  RecognizerType recognizer_;

  virtual void updateParameters(const typename ActionType::_action_goal_type::_goal_type& goal,
                                typename RecognizerDataType::Parameters& parameters) = 0;
  virtual void updateResult(const typename RecognizerDataType::Result& result,
                            typename ActionType::_action_result_type::_result_type& action_result) = 0;

  actionlib::SimpleActionServer<ActionType> action_server_;
  double execute_period_ = 0.01;
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
      action_server_.setAborted(action_result, msg);
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
      action_server_.setAborted(action_result, msg);
      return;
    }

    ROS_INFO("Recognizing ..");
    recognizer_.recognize();
    try
    {
      while (ros::ok() && recognizer_.isRecognizing())
      {
        if (action_server_.isPreemptRequested() && !recognizer_.isPreempting())
        {
          ROS_WARN("Preempt requested");
          recognizer_.preempt();
        }
        ros::Duration(execute_period_).sleep();
      }
    }
    catch (const std::exception& e)
    {
      std::string msg = "Recognize error: " + std::string(e.what());
      ROS_ERROR("%s", msg.c_str());
      action_server_.setAborted(action_result, msg);
      return;
    }
    ROS_INFO("Recognizing done");

    auto result = recognizer_.getResult();
    ROS_INFO("Result: %s", toString(result).c_str());

    updateResult(result, action_result);
    if (action_server_.isPreemptRequested())
    {
      action_server_.setPreempted(action_result);
    }
    else
    {
      action_server_.setSucceeded(action_result);
    }
  }
};
}  // namespace picovoice_driver
