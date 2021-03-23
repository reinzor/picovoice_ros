#pragma once

#include <map>
#include <ostream>
#include <picovoice.h>
#include <pv_rhino.h>
#include <string>
#include <vector>

#include "./recognizer.h"
#include "./util.h"

namespace picovoice_driver
{
struct RhinoRecognizerData
{
  struct Parameters
  {
    std::string model_path_;
    std::string context_path_;
    double sensitivity_ = 0.5;
  };

  struct Result
  {
    bool is_understood_ = false;
    std::string intent_;

    struct KeyValue
    {
      KeyValue(const std::string& key, const std::string& value);

      std::string key_;
      std::string value_;
    };

    std::vector<KeyValue> slots_;
  };
};
std::ostream& operator<<(std::ostream& os, const RhinoRecognizerData::Parameters& p);
std::ostream& operator<<(std::ostream& os, const RhinoRecognizerData::Result::KeyValue& kv);
std::ostream& operator<<(std::ostream& os, const RhinoRecognizerData::Result& r);

class RhinoRecognizer : public RecognizerT<RhinoRecognizerData>
{
public:
  ~RhinoRecognizer();

  void configure(const RhinoRecognizerData::Parameters& parameters) override;
  RhinoRecognizerData::Result getResult() override;

private:
  RecordSettings getRecordSettings() override;
  void recognizeInit() override;

  bool recognizeProcess(int16_t* frames) override;

  pv_rhino_t* rhino_ = NULL;
};
}  // namespace picovoice_driver
