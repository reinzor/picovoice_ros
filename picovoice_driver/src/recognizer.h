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

#include <atomic>
#include <memory>
#include <thread>

namespace picovoice_driver
{
//!
//! \brief The Recognizer class used for recognizing something out of audio data
//!
class Recognizer
{
public:
  void initialize(const std::string& record_directory, double record_timeout);

  //!
  //! \brief recognize Recognize something from an audio input stream
  //!
  void recognize();

  //!
  //! \brief preempt Preempt the recognition
  //!
  void preempt();

  //!
  //! \brief isPreempting Whether a preempt was requested
  //!
  bool isPreempting();

  //!
  //! \brief isRecognizing Whether the recognize method is running
  //! \return True if recognizing, False otherwise
  //!
  bool isRecognizing();

protected:
  bool initialized_ = false;
  std::string record_directory_;
  double record_timeout_;

  struct RecordSettings
  {
    size_t sample_rate_ = 0;
    size_t frame_length_ = 0;
  };

  std::string recognize_thread_exception_string_;
  std::shared_ptr<std::thread> recognize_thread_;
  void recognizeThread();
  void recognizeThreadCatchException();
  virtual RecordSettings getRecordSettings() = 0;
  virtual void recognizeInit() = 0;
  virtual bool recognizeProcess(int16_t* frames) = 0;

private:
  std::atomic<bool> preempt_requested_ = ATOMIC_VAR_INIT(false);
  std::atomic<bool> is_recognizing_ = ATOMIC_VAR_INIT(false);
};

template <typename RecognizerDataType>
class RecognizerT : public Recognizer
{
public:
  virtual void configure(const typename RecognizerDataType::Parameters& parameters) = 0;
  virtual typename RecognizerDataType::Result getResult() = 0;
};
}  // namespace picovoice_driver
