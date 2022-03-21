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
        parameters.keywords_ = {{"keyword", std::string(optarg)}};
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
  recognizer.initialize("/tmp/picovoice_driver/test_porcupine", 10.);
  recognizer.configure(parameters);

  std::cout << "Configure with parameters " << parameters << std::endl;

  recognizer.recognize();

  while (recognizer.isRecognizing())
  {
    std::cout << "Recognizing .." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::cout << "Result: " << recognizer.getResult() << std::endl;

  return 0;
}
