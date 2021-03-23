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
  struct Parameters
  {
    std::string model_path_;
    std::string keyword_path_;
    double sensitivity_ = 0.5;
  };

  struct Result
  {
    bool is_understood_ = false;
    std::string keyword_;
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

  std::vector<std::string> keywords_;
  PorcupineRecognizerData::Result result_;
  pv_porcupine_t* porcupine_ = NULL;
};
}  // namespace picovoice_driver
