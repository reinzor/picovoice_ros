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

#include <map>
#include <ostream>
#include <picovoice.h>
#include <pv_porcupine.h>
#include <string>
#include <vector>

#include "./recognizer.h"

namespace picovoice_driver
{
struct PorcupineRecognizerData
{
  struct Parameters : RecognizerData::Parameters
  {
    //!
    //! \brief keywords_ Map from a keyword names to a keyword.ppn paths
    //!
    std::map<std::string, std::string> keywords_;

    //!
    //! \brief sensitivity_ Recognizer sensitivity
    //!
    double sensitivity_ = 0.5;
  };

  struct Result
  {
    //!
    //! \brief is_understood_ Whether the recognizer understood a keyword
    //!
    bool is_understood_ = false;

    //!
    //! \brief keyword_name_ Detected keyword name
    //!
    std::string keyword_name_;
  };
};
std::ostream& operator<<(std::ostream& os, const PorcupineRecognizerData::Parameters& p);
std::ostream& operator<<(std::ostream& os, const PorcupineRecognizerData::Result& r);

class PorcupineRecognizer : public RecognizerT<PorcupineRecognizerData>
{
public:
  ~PorcupineRecognizer();

  void configure(const PorcupineRecognizerData::Parameters& parameters) override;
  PorcupineRecognizerData::Result getResult() override;

private:
  RecordSettings getRecordSettings() override;
  void recognizeInit() override;

  bool recognizeProcess(int16_t* frames) override;

  std::vector<std::string> keyword_names_;
  std::vector<const char*> keyword_paths_;
  std::vector<float> keyword_sensitivities_;

  PorcupineRecognizerData::Result result_;
  pv_porcupine_t* porcupine_ = NULL;
};
}  // namespace picovoice_driver
