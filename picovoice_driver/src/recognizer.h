#pragma once

#include <atomic>
#include <cstdint>
#include <portaudio.h>
#include <stddef.h>

namespace picovoice_driver
{
//!
//! \brief The Recognizer class used for recognizing something out of audio data
//!
class Recognizer
{
public:
  //!
  //! \brief recognize Recognize something from an audio input stream
  //!
  void recognize();

  //!
  //! \brief preempt Preempt the recognition (thread safe)
  //!
  void preempt();

protected:
  struct RecordSettings
  {
    size_t sample_rate_ = 0;
    size_t frame_length_ = 0;
  };

  virtual RecordSettings getRecordSettings() = 0;
  virtual void recognizeInit() = 0;

  virtual bool recognizeProcess(int16_t* frames) = 0;

private:
  std::atomic<bool> preempt_requested_;
};

template <typename RecognizerDataType>
class RecognizerT : public Recognizer
{
public:
  virtual void configure(const typename RecognizerDataType::Parameters& parameters) = 0;
  virtual typename RecognizerDataType::Result getResult() = 0;
};
}  // namespace picovoice_driver
