#include <stdexcept>

#include "./rhino_recognizer.h"

namespace picovoice_driver
{
std::ostream& operator<<(std::ostream& os, const RhinoRecognizerData::Parameters& p)
{
  os << "Parameters(context_path=" << p.context_path_ << ", model_path=" << p.model_path_
     << ", sensitivity=" << p.sensitivity_ << ")";
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
  pv_status_t status =
      pv_rhino_init(parameters.model_path_.data(), parameters.context_path_.data(), parameters.sensitivity_, &rhino_);
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
}

RhinoRecognizerData::Result RhinoRecognizer::getResult()
{
  RhinoRecognizerData::Result result;
  pv_status_t status = pv_rhino_is_understood(rhino_, &result.is_understood_);
  if (status != PV_STATUS_SUCCESS)
  {
    throw std::runtime_error("Rhino is understood failed: " + std::string(pv_status_to_string(status)));
  }

  if (result.is_understood_)
  {
    const char* intent = NULL;
    int32_t num_slots = 0;
    const char** slots = NULL;
    const char** values = NULL;
    status = pv_rhino_get_intent(rhino_, &intent, &num_slots, &slots, &values);
    if (status != PV_STATUS_SUCCESS)
    {
      throw std::runtime_error("Rhino get intent failed: " + std::string(pv_status_to_string(status)));
    }
    result.intent_ = std::string(intent);
    for (int32_t i = 0; i < num_slots; ++i)
    {
      result.slots_.emplace_back(std::string(slots[i]), std::string(values[i]));
    }
  }
  return result;
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
  pv_rhino_reset(rhino_);
}

bool RhinoRecognizer::recognizeProcess(int16_t* frames)
{
  bool is_finalized = false;
  pv_status_t status = pv_rhino_process(rhino_, frames, &is_finalized);
  if (status != PV_STATUS_SUCCESS)
  {
    throw std::runtime_error("Rhino process failed: " + std::string(pv_status_to_string(status)));
  }
  return is_finalized;
}
}  // namespace picovoice_driver
