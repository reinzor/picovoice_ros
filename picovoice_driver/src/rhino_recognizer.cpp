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

#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <yaml-cpp/yaml.h>

#include "./rhino_recognizer.h"

namespace picovoice_driver
{
std::ostream& operator<<(std::ostream& os, const RhinoRecognizerData::Parameters& p)
{
  os << "Parameters(access_key=" << p.access_key_ << ", model_path=" << p.model_path_
     << ", context_path=" << p.context_path_ << ", sensitivity=" << p.sensitivity_ << ")";
  return os;
}

RhinoRecognizerData::Result::KeyValue::KeyValue(const std::string& key, const std::string& value)
  : key_(key), value_(value)
{
}

std::ostream& operator<<(std::ostream& os, const RhinoRecognizerData::Result::KeyValue& kv)
{
  os << "{" << kv.key_ << "=" << kv.value_ << "}";
  return os;
}

std::ostream& operator<<(std::ostream& os, const RhinoRecognizerData::Result& r)
{
  os << "Result(is_understood=" << r.is_understood_ << ", intent=" << r.intent_ << ", slots=" << toString(r.slots_)
     << ")";
  return os;
}

RhinoRecognizer::~RhinoRecognizer()
{
  if (rhino_ != NULL)
  {
    pv_rhino_delete(rhino_);
  }
}

void RhinoRecognizer::configure(const RhinoRecognizerData::Parameters& parameters)
{
  if (rhino_ != NULL)
  {
    pv_rhino_delete(rhino_);
  }

  pv_status_t status =
      pv_rhino_init(parameters.access_key_.data(), parameters.model_path_.data(), parameters.context_path_.data(),
                    static_cast<float>(parameters.sensitivity_), parameters.require_endpoint_, &rhino_);
  if (status != PV_STATUS_SUCCESS)
  {
    throw std::runtime_error("Failed to initialize picovoice rhino with parameters " + toString(parameters) + ": " +
                             std::string(pv_status_to_string(status)));
  }
  const char* context_info = NULL;
  status = pv_rhino_context_info(rhino_, &context_info);
  if (status != PV_STATUS_SUCCESS)
  {
    throw std::runtime_error("Failed to get rhino context info: " + std::string(pv_status_to_string(status)));
  }

  auto yaml = YAML::Load(context_info);
  if (!yaml["context"])
  {
    throw std::runtime_error("Context missing key 'context': " + std::string(context_info));
  }
  if (!yaml["context"]["expressions"])
  {
    throw std::runtime_error("Context missing key 'context/expressions': " + std::string(context_info));
  }

  std::vector<std::string> context_intents;
  for (const auto& kv : yaml["context"]["expressions"])
  {
    context_intents.push_back(kv.first.as<std::string>());
  }

  for (const auto& intent : parameters.intents_)
  {
    if (std::find(context_intents.begin(), context_intents.end(), intent) == context_intents.end())
    {
      throw std::runtime_error("Intent '" + intent +
                               "' not available in context. Available intents: " + toString(context_intents));
    }
  }

  intents_ = parameters.intents_;
}

RhinoRecognizerData::Result RhinoRecognizer::getResult()
{
  return result_;
}

Recognizer::RecordSettings RhinoRecognizer::getRecordSettings()
{
  Recognizer::RecordSettings settings;
  settings.frame_length_ = pv_rhino_frame_length();
  settings.sample_rate_ = pv_sample_rate();
  return settings;
}

void RhinoRecognizer::recognizeInit()
{
  result_ = RhinoRecognizerData::Result();
  pv_rhino_reset(rhino_);
}

bool RhinoRecognizer::recognizeProcess(int16_t* frames)
{
  bool is_finalized = false;
  auto process_status = pv_rhino_process(rhino_, frames, &is_finalized);
  if (process_status != PV_STATUS_SUCCESS)
  {
    throw std::runtime_error("Rhino process failed: " + std::string(pv_status_to_string(process_status)));
  }

  if (is_finalized)
  {
    bool is_understood;
    auto is_understood_status = pv_rhino_is_understood(rhino_, &is_understood);
    if (is_understood_status != PV_STATUS_SUCCESS)
    {
      throw std::runtime_error("Rhino is understood failed: " + std::string(pv_status_to_string(is_understood_status)));
    }

    if (!is_understood)
    {
      pv_rhino_reset(rhino_);
      return false;
    }

    const char* intent_char_array = NULL;
    int32_t num_slots = 0;
    const char** slots = NULL;
    const char** values = NULL;

    auto intent_status = pv_rhino_get_intent(rhino_, &intent_char_array, &num_slots, &slots, &values);
    if (intent_status != PV_STATUS_SUCCESS)
    {
      throw std::runtime_error("Rhino get intent failed: " + std::string(pv_status_to_string(is_understood_status)));
    }

    std::string intent(intent_char_array);
    if (!intents_.empty() && std::find(intents_.begin(), intents_.end(), std::string(intent)) == intents_.end())
    {
      pv_rhino_reset(rhino_);
      return false;
    }

    result_.is_understood_ = is_understood;
    result_.intent_ = intent;
    for (int32_t i = 0; i < num_slots; ++i)
    {
      result_.slots_.emplace_back(std::string(slots[i]), std::string(values[i]));
    }
    return true;
  }
  return false;
}
}  // namespace picovoice_driver
