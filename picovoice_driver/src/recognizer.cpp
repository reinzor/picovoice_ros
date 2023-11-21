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

#include <cmath>
#include <boost/filesystem.hpp>
#include <chrono>
extern "C" {
#include <pv_recorder.h>
}
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
  try
  {
    recognizeInit();
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("recognizeInit failed: " + std::string(e.what()));
  }

  RecordSettings record_settings;
  try
  {
    record_settings = getRecordSettings();
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("getRecordSettings failed: " + std::string(e.what()));
  }

  pv_recorder_t* recorder = NULL;
  pv_recorder_status_t recorder_status =
      pv_recorder_init(record_settings.frame_length_, -1, 10, &recorder);
  if (recorder_status != PV_RECORDER_STATUS_SUCCESS)
  {
    throw std::runtime_error("Failed to initialize device with " +
                             std::string(pv_recorder_status_to_string(recorder_status)));
  }

  recorder_status = pv_recorder_start(recorder);
  if (recorder_status != PV_RECORDER_STATUS_SUCCESS)
  {
    pv_recorder_delete(recorder);
    throw std::runtime_error("Failed to start device with " +
                             std::string(pv_recorder_status_to_string(recorder_status)));
  }

  bool is_finalized = false;
  size_t frame_index = 0;
  size_t total_frames = std::ceil((record_timeout_ * record_settings.sample_rate_) / record_settings.frame_length_);
  std::vector<int16_t> pcm(record_settings.frame_length_);
  std::vector<int16_t> record_buffer(record_settings.frame_length_ * total_frames, 0);
  while (frame_index < total_frames && !is_finalized && !preempt_requested_.load())
  {
    recorder_status = pv_recorder_read(recorder, pcm.data());
    if (recorder_status != PV_RECORDER_STATUS_SUCCESS)
    {
      is_recognizing_.store(false);
      pv_recorder_delete(recorder);
      throw std::runtime_error("Failed to read with " + std::string(pv_recorder_status_to_string(recorder_status)));
    }

    try
    {
      is_finalized = recognizeProcess(pcm.data());
    }
    catch (const std::exception& e)
    {
      is_recognizing_.store(false);
      pv_recorder_delete(recorder);
      throw std::runtime_error("recognizeProcess failed: " + std::string(e.what()));
    }

    std::copy(pcm.begin(), pcm.end(), record_buffer.begin() + frame_index * pcm.size());
    ++frame_index;
  }

  recorder_status = pv_recorder_stop(recorder);
  if (recorder_status != PV_RECORDER_STATUS_SUCCESS)
  {
    is_recognizing_.store(false);
    pv_recorder_delete(recorder);
    throw std::runtime_error("Failed to stop device with " +
                             std::string(pv_recorder_status_to_string(recorder_status)));
  }

  if (!record_directory_.empty())
  {
    writeWav(record_buffer, frame_index * record_settings.frame_length_, record_settings.sample_rate_,
             record_directory_);
  }

  pv_recorder_delete(recorder);
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

void Recognizer::initialize(const std::string& record_directory, double record_timeout)
{
  record_directory_ = record_directory;
  record_timeout_ = record_timeout;
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
  preempt_requested_.store(false);
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
