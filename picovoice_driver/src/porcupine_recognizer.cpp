#include <stdexcept>

#include "./porcupine_recognizer.h"
#include "./util.h"

namespace picovoice_driver
{
std::ostream& operator<<(std::ostream& os, const PorcupineRecognizerData::Parameters& p)
{
  os << "Parameters(keyword_path=" << p.keyword_path_ << ", model_path=" << p.model_path_
     << ", sensitivity=" << p.sensitivity_ << ")";
  return os;
}

std::ostream& operator<<(std::ostream& os, const PorcupineRecognizerData::Result& r)
{
  os << "Result(is_understood=" << r.is_understood_ << ", keyword=" << r.keyword_ << ")";
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
  const char* keyword = parameters.keyword_path_.data();
  float sensitivity = static_cast<float>(parameters.sensitivity_);
  pv_status_t status = pv_porcupine_init(parameters.model_path_.data(), 1, &keyword, &sensitivity, &porcupine_);
  if (status != PV_STATUS_SUCCESS)
  {
    throw std::runtime_error("Failed to initialize picovoice porcupine with parameters " + toString(parameters) + ":" +
                             std::string(pv_status_to_string(status)));
  }
  keywords_ = { parameters.keyword_path_ };
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
  if (static_cast<size_t>(keyword_index) >= keywords_.size())
  {
    throw std::runtime_error("Keyword index " + std::to_string(keyword_index) + " out of bound for keywords " +
                             toString(keywords_));
  }

  result_.is_understood_ = true;
  result_.keyword_ = keywords_.at(keyword_index);
  return true;
}
}  // namespace picovoice_driver
