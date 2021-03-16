#pragma once

#include <cstdint>
#include <portaudio.h>
#include <stddef.h>

namespace picovoice_driver
{
class Recognizer
{
public:
  void recognize();

protected:
  struct RecordSettings
  {
    size_t sample_rate_ = 0;
    size_t frame_length_ = 0;
  };

  virtual RecordSettings getRecordSettings() = 0;
  virtual void recognizeInit() = 0;

  virtual bool recognizeProcess(int16_t* frames) = 0;
};

template <typename RecognizerDataType>
class RecognizerT : public Recognizer
{
public:
  virtual void configure(const typename RecognizerDataType::Parameters& parameters) = 0;
  virtual typename RecognizerDataType::Result getResult() = 0;
};
}  // namespace picovoice_driver
