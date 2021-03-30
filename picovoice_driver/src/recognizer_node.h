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

#pragma once

#include <actionlib/server/simple_action_server.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <ros/node_handle.h>
#include <string>

#include "./ros_util.h"
#include "./util.h"

namespace picovoice_driver
{
constexpr double EXECUTE_PERIOD = 0.01;

template <typename RecognizerDataType, typename RecognizerType, typename ActionType>
class RecognizerNode
{
public:
  explicit RecognizerNode(const std::string& name, const std::string& action_name,
                          const typename RecognizerDataType::Parameters& parameters)
    : action_server_(action_name, boost::bind(&RecognizerNode::executeCallback, this, _1), false)
    , parameters_(parameters)
  {
    ros::NodeHandle local_nh("~");
    auto record_directory = local_nh.param("record_directory", defaultRecordDirectory(name));
    auto record_timeout = local_nh.param("record_timeout", 300.);
    recognizer_.initialize(record_directory, record_timeout);

    dynamic_reconfigure_server_.registerVariable<double>("sensitivity", &parameters_.sensitivity_,
                                                         "Recognizer sensitivity", 0., 1.);
    dynamic_reconfigure_server_.publishServicesTopics();

    action_server_.start();
    ROS_INFO("RecognizerNode(name=%s, action_name=%s, record_directory=%s, record_timeout=%.2f) initialized with "
             "parameters %s",
             name.c_str(), action_name.c_str(), record_directory.c_str(), record_timeout,
             toString(parameters_).c_str());
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
        ros::Duration(EXECUTE_PERIOD).sleep();
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
