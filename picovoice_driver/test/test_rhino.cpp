#include <getopt.h>
#include <signal.h>
#include <chrono>
#include <thread>
#include <iostream>

#include "../src/rhino_recognizer.h"

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

  picovoice_driver::RhinoRecognizerData::Parameters parameters;

  int c;
  while ((c = getopt_long(argc, argv, "m:c:a:s:r", long_options, NULL)) != -1)
  {
    switch (c)
    {
      case 'm':
        parameters.model_path_ = std::string(optarg);
        break;
      case 'c':
        parameters.context_path_ = std::string(optarg);
        break;
      case 't':
        parameters.sensitivity_ = strtof(optarg, NULL);
        break;
      case 'a':
        parameters.access_key_ = std::string(optarg);
        break;
      case 'r':
        parameters.require_endpoint_ = true;
        break;
      default:
        exit(1);
    }
  }

  if (parameters.model_path_.empty() || parameters.context_path_.empty() || parameters.access_key_.empty())
  {
    fprintf(stderr, "Usage : %s -m MODEL_PATH -c CONTEXT_PATH -a ACCESS_KEY [-s SENSTIVITY -r (require endpoint)]\n", argv[0]);
    exit(1);
  }

  picovoice_driver::RhinoRecognizer recognizer;
  recognizer.initialize("/tmp/picovoice_driver/test_rhino", 100.);
  recognizer.configure(parameters);

  std::cout << "Configure with parameters " << parameters << std::endl;

  while (true)
  {
    recognizer.recognize();
    while (recognizer.isRecognizing())
    {
//      std::cout << "Recognizing .." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (recognizer.getResult().is_understood_)
    {
      std::cout << "Result: " << recognizer.getResult() << std::endl;
    }
  }

  return 0;
}
