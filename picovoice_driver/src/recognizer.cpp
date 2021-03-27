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

#include <boost/filesystem.hpp>
#include <chrono>
#include <portaudio.h>
#include <sndfile.h>
#include <stdexcept>
#include <vector>

#include "./recognizer.h"

namespace picovoice_driver
{
std::string getEpochStamp()
{
  using namespace std::chrono;
  unsigned long ms_since_epoch = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
  unsigned long seconds_since_epoch = ms_since_epoch / 1e3;
  unsigned long ms_decimals_since_epoch = ms_since_epoch - seconds_since_epoch * 1e3;
  return std::to_string(seconds_since_epoch) + "." + std::to_string(ms_decimals_since_epoch);
}

void writeWav(const std::vector<int16_t>& buffer, size_t buffer_size, size_t sample_rate, const std::string& directory)
{
  boost::filesystem::create_directories(directory);
  std::string filename = directory + "/recording-" + getEpochStamp() + ".wav";

  SF_INFO sfinfo;
  sfinfo.channels = 1;
  sfinfo.samplerate = sample_rate;
  sfinfo.format = SF_FORMAT_WAV | SF_FORMAT_PCM_16;
  SNDFILE* outfile = sf_open(filename.c_str(), SFM_WRITE, &sfinfo);
  if (outfile != nullptr)
  {
    sf_write_short(outfile, buffer.data(), buffer_size);
    sf_write_sync(outfile);
    sf_close(outfile);
  }
}

void Recognizer::recognizeThread()
{
  preempt_requested_.store(false);
  is_recognizing_.store(true);

  PaError err = Pa_Initialize();
  if (err != paNoError)
  {
    is_recognizing_.store(false);
    throw std::runtime_error("Failed to initialize PortAudio");
  }

  PaStreamParameters input_parameters;
  input_parameters.device = Pa_GetDefaultInputDevice();
  if (input_parameters.device == paNoDevice)
  {
    Pa_Terminate();
    is_recognizing_.store(false);
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
    is_recognizing_.store(false);
    throw std::runtime_error("getRecordSettings failed: " + std::string(e.what()));
  }

  try
  {
    recognizeInit();
  }
  catch (const std::exception& e)
  {
    Pa_Terminate();
    is_recognizing_.store(false);
    throw std::runtime_error("recognizeInit failed: " + std::string(e.what()));
  }

  PaStream* stream;
  err = Pa_OpenStream(&stream, &input_parameters, NULL, record_settings.sample_rate_, paFramesPerBufferUnspecified,
                      paClipOff, NULL, NULL);
  if (err != paNoError)
  {
    Pa_Terminate();
    is_recognizing_.store(false);
    throw std::runtime_error("Could not open stream");
  }

  err = Pa_StartStream(stream);
  if (err != paNoError)
  {
    Pa_Terminate();
    is_recognizing_.store(false);
    throw std::runtime_error("Could not start stream");
  }

  bool is_finalized = false;
  size_t sample_index = 0;
  size_t total_samples = max_record_length_ * record_settings.sample_rate_;
  std::vector<int16_t> recorded_samples(total_samples * sizeof(int16_t));
  while (sample_index < total_samples && !is_finalized && !preempt_requested_.load())
  {
    err = Pa_ReadStream(stream, &recorded_samples.data()[sample_index], record_settings.frame_length_);
    if (err)
    {
      Pa_Terminate();
      is_recognizing_.store(false);
      throw std::runtime_error("Could not read stream");
    }

    try
    {
      is_finalized = recognizeProcess(&recorded_samples[sample_index]);
    }
    catch (const std::exception& e)
    {
      Pa_Terminate();
      is_recognizing_.store(false);
      throw std::runtime_error("recognizeProcess failed: " + std::string(e.what()));
    }
    sample_index += record_settings.frame_length_;
  }

  Pa_Terminate();

  if (!record_directory_.empty())
  {
    writeWav(recorded_samples, sample_index, record_settings.sample_rate_, record_directory_);
  }

  is_recognizing_.store(false);
}

void Recognizer::recognizeThreadCatchException()
{
  try
  {
    recognizeThread();
  }
  catch (const std::exception& e)
  {
    recognize_thread_exception_string_ = std::string(e.what());
  }
}

void Recognizer::initialize(const std::string& record_directory, double max_record_length)
{
  record_directory_ = record_directory;
  max_record_length_ = max_record_length;
  initialized_ = true;
}

void Recognizer::recognize()
{
  if (!initialized_)
  {
    throw std::runtime_error("Recognizer not initialized");
  }

  if (recognize_thread_ != nullptr)
  {
    throw std::runtime_error("Already recognizing");
  }
  recognize_thread_exception_string_.clear();
  is_recognizing_.store(true);
  recognize_thread_.reset(new std::thread(&Recognizer::recognizeThreadCatchException, this));
}

void Recognizer::preempt()
{
  if (!initialized_)
  {
    throw std::runtime_error("Recognizer not initialized");
  }
  preempt_requested_.store(true);
}

bool Recognizer::isPreempting()
{
  if (!initialized_)
  {
    throw std::runtime_error("Recognizer not initialized");
  }
  return preempt_requested_.load();
}

bool Recognizer::isRecognizing()
{
  if (!initialized_)
  {
    throw std::runtime_error("Recognizer not initialized");
  }
  bool is_recognizing = is_recognizing_.load();
  if (!is_recognizing)
  {
    recognize_thread_->join();
    recognize_thread_.reset();
    if (!recognize_thread_exception_string_.empty())
    {
      throw std::runtime_error("Recognize thread exception: " + recognize_thread_exception_string_);
    }
  }
  return is_recognizing;
}
}  // namespace picovoice_driver
