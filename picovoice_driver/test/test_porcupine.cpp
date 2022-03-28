/*
 * Copyright 2022, Rein Appeldoorn
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

#include <getopt.h>
#include <signal.h>
#include <chrono>
#include <thread>
#include <iostream>

#include "../src/porcupine_recognizer.h"

static struct option long_options[] = {
  { "model_path", required_argument, NULL, 'm' },
  { "keyword_path", required_argument, NULL, 'k' },
  { "access_key", required_argument, NULL, 'a' },
  { "sensitivity", required_argument, NULL, 's' },
};

void interrupt_handler(int)
{
  exit(1);
}

int main(int argc, char** argv)
{
  signal(SIGINT, interrupt_handler);

  picovoice_driver::PorcupineRecognizerData::Parameters parameters;

  int c;
  while ((c = getopt_long(argc, argv, "m:k:a:s", long_options, NULL)) != -1)
  {
    switch (c)
    {
      case 'm':
        parameters.model_path_ = std::string(optarg);
        break;
      case 'k':
        parameters.keywords_ = { { "keyword", std::string(optarg) } };
        break;
      case 't':
        parameters.sensitivity_ = strtof(optarg, NULL);
        break;
      case 'a':
        parameters.access_key_ = std::string(optarg);
        break;
      default:
        exit(1);
    }
  }

  if (parameters.model_path_.empty() || parameters.keywords_.empty() || parameters.access_key_.empty())
  {
    fprintf(stderr, "Usage : %s -m MODEL_PATH -k KEYWORD_PATH -a ACCESS_KEY [-s SENSTIVITY]\n", argv[0]);
    exit(1);
  }

  picovoice_driver::PorcupineRecognizer recognizer;
  recognizer.initialize("/tmp/picovoice_driver/test_porcupine", 5.);
  recognizer.configure(parameters);

  std::cout << "Configure with parameters " << parameters << std::endl;

  while (true)
  {
    recognizer.recognize();
    while (recognizer.isRecognizing())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (recognizer.getResult().is_understood_)
    {
      std::cout << "Result: " << recognizer.getResult() << std::endl;
    }
    else
    {
      std::cout << "--" << std::endl;
    }
  }

  return 0;
}
