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

#include <stdexcept>

#include "./porcupine_recognizer.h"
#include "./util.h"

namespace picovoice_driver
{
std::ostream& operator<<(std::ostream& os, const PorcupineRecognizerData::Parameters& p)
{
  os << "Parameters(keywords_=" << toString(p.keywords_) << ", model_path=" << p.model_path_
     << ", sensitivity=" << p.sensitivity_ << ")";
  return os;
}

std::ostream& operator<<(std::ostream& os, const PorcupineRecognizerData::Result& r)
{
  os << "Result(is_understood=" << r.is_understood_ << ", keyword=" << r.keyword_name_ << ")";
  return os;
}

PorcupineRecognizer::~PorcupineRecognizer()
{
  if (porcupine_ != NULL)
  {
    pv_porcupine_delete(porcupine_);
  }
}

void PorcupineRecognizer::configure(const PorcupineRecognizerData::Parameters& parameters)
{
  keyword_names_.clear();
  keyword_paths_.clear();
  keyword_sensitivities_.clear();
  for (const auto& kv : parameters.keywords_)
  {
    keyword_names_.push_back(kv.first);
    keyword_paths_.push_back(kv.second.c_str());
    keyword_sensitivities_.push_back(static_cast<float>(parameters.sensitivity_));
  }

  pv_status_t status = pv_porcupine_init(parameters.model_path_.data(), keyword_names_.size(), keyword_paths_.data(),
                                         keyword_sensitivities_.data(), &porcupine_);
  if (status != PV_STATUS_SUCCESS)
  {
    throw std::runtime_error("Failed to initialize picovoice porcupine with parameters " + toString(parameters) + ":" +
                             std::string(pv_status_to_string(status)));
  }
}

PorcupineRecognizerData::Result PorcupineRecognizer::getResult()
{
  return result_;
}

Recognizer::RecordSettings PorcupineRecognizer::getRecordSettings()
{
  Recognizer::RecordSettings settings;
  settings.frame_length_ = pv_porcupine_frame_length();
  settings.sample_rate_ = pv_sample_rate();
  return settings;
}

void PorcupineRecognizer::recognizeInit()
{
  result_ = PorcupineRecognizerData::Result();
}

bool PorcupineRecognizer::recognizeProcess(int16_t* frames)
{
  int32_t keyword_index = -1;
  pv_status_t status = pv_porcupine_process(porcupine_, frames, &keyword_index);
  if (status != PV_STATUS_SUCCESS)
  {
    throw std::runtime_error("Porcupine process failed: " + std::string(pv_status_to_string(status)));
  }
  if (keyword_index < 0)
  {
    return false;
  }
  if (static_cast<size_t>(keyword_index) >= keyword_names_.size())
  {
    throw std::runtime_error("Keyword index " + std::to_string(keyword_index) + " out of bound for keywords " +
                             toString(keyword_names_));
  }

  result_.is_understood_ = true;
  result_.keyword_name_ = keyword_names_.at(keyword_index);
  return true;
}
}  // namespace picovoice_driver
