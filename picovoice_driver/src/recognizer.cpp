#include <stdexcept>
#include <vector>

#include "./recognizer.h"

namespace picovoice_driver
{
void Recognizer::recognize()
{
  PaError err = Pa_Initialize();
  if (err != paNoError)
  {
    throw std::runtime_error("Failed to initialize PortAudio");
  }

  PaStreamParameters input_parameters;
  input_parameters.device = Pa_GetDefaultInputDevice();
  if (input_parameters.device == paNoDevice)
  {
    Pa_Terminate();
    throw std::runtime_error("Could not get default input device");
  }
  input_parameters.channelCount = 1;
  input_parameters.sampleFormat = paInt16;
  input_parameters.suggestedLatency = Pa_GetDeviceInfo(input_parameters.device)->defaultLowInputLatency;
  input_parameters.hostApiSpecificStreamInfo = NULL;

  RecordSettings record_settings;
  try
  {
    record_settings = getRecordSettings();
  }
  catch (const std::exception& e)
  {
    Pa_Terminate();
    throw std::runtime_error("getRecordSettings failed: " + std::string(e.what()));
  }

  try
  {
    recognizeInit();
  }
  catch (const std::exception& e)
  {
    Pa_Terminate();
    throw std::runtime_error("recognizeInit failed: " + std::string(e.what()));
  }

  PaStream* stream;
  err = Pa_OpenStream(&stream, &input_parameters, NULL, record_settings.sample_rate_, paFramesPerBufferUnspecified,
                      paClipOff, NULL, NULL);
  if (err != paNoError)
  {
    Pa_Terminate();
    throw std::runtime_error("Could not open stream");
  }

  err = Pa_StartStream(stream);
  if (err != paNoError)
  {
    Pa_Terminate();
    throw std::runtime_error("Could not start stream");
  }

  bool is_finalized = false;
  double num_seconds = 10;
  size_t sample_index = 0;
  size_t total_samples = num_seconds * record_settings.sample_rate_;
  std::vector<int16_t> recorded_samples(total_samples * sizeof(int16_t));
  while (sample_index < total_samples && !is_finalized)
  {
    err = Pa_ReadStream(stream, &recorded_samples.data()[sample_index], record_settings.frame_length_);
    if (err)
    {
      Pa_Terminate();
      throw std::runtime_error("Could not read stream");
    }

    try
    {
      is_finalized = recognizeProcess(&recorded_samples[sample_index]);
    }
    catch (const std::exception& e)
    {
      Pa_Terminate();
      throw std::runtime_error("recognizeProcess failed: " + std::string(e.what()));
    }
    sample_index += record_settings.frame_length_;
  }

  Pa_Terminate();
}
}  // namespace picovoice_driver
