/** @file paex_record.c
    @ingroup examples_src
    @brief Record input into an array; Save array to a file; Playback recorded data.
    @author Phil Burk  http://www.softsynth.com
*/
/*
 * $Id$
 *
 * This program uses the PortAudio Portable Audio Library.
 * For more information see: http://www.portaudio.com
 * Copyright (c) 1999-2000 Ross Bencina and Phil Burk
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 * The text above constitutes the entire PortAudio license; however,
 * the PortAudio community also makes the following non-binding requests:
 *
 * Any person wishing to distribute modifications to the Software is
 * requested to send the modifications to the original developer so that
 * they can be incorporated into the canonical version. It is also
 * requested that these non-binding requests be included along with the
 * license above.
 */

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "portaudio.h"
#include <iostream>
#include <thread>
#include <pv_rhino.h>

#define NUM_SECONDS (5)

typedef struct
{
  int sampleIndex; /* Index into sample array. */
  int maxSampleIndex;
  int16_t* recordedSamples;
} paTestData;

/*******************************************************************/
int main(int argc, char** argv)
{
  if (argc != 3)
  {
    fprintf(stderr, "usage : %s model_path context_path\n", argv[0]);
    exit(1);
  }

  const char* model_path = argv[1];
  const char* context_path = argv[2];

  pv_rhino_t* rhino = NULL;
  pv_status_t status = pv_rhino_init(model_path, context_path, 0.5f, &rhino);

  PaStreamParameters inputParameters, outputParameters;
  PaStream* stream;
  PaError err = paNoError;
  paTestData data;
  int i;
  int totalSamples;
  int numSamples;
  int numBytes;
  double average;

  printf("patest_record.c\n");
  fflush(stdout);

  std::cout << "Sample rate: " << pv_sample_rate() << std::endl;
  std::cout << "Frame length: " << pv_rhino_frame_length() << std::endl;
  std::cout << "Frame length (seconds): " << static_cast<double>(pv_rhino_frame_length()) / pv_sample_rate()
            << std::endl;

  data.maxSampleIndex = totalSamples = NUM_SECONDS * pv_sample_rate(); /* Record for a few seconds. */
  data.sampleIndex = 0;
  numSamples = totalSamples;
  numBytes = numSamples * sizeof(int16_t);
  data.recordedSamples = (int16_t*)malloc(numBytes); /* From now on, recordedSamples is initialised. */
  if (data.recordedSamples == NULL)
  {
    printf("Could not allocate record array.\n");
    exit(1);
  }
  for (i = 0; i < numSamples; i++)
    data.recordedSamples[i] = 0;

  err = Pa_Initialize();
  if (err != paNoError)
    exit(1);

  const char* context_info = NULL;
  status = pv_rhino_context_info(rhino, &context_info);
  if (status != PV_STATUS_SUCCESS)
  {
    fprintf(stderr, "'pv_rhino_context_info' failed with '%s'\n", pv_status_to_string(status));
    exit(1);
  }
  fprintf(stdout, "%s\n\n", context_info);

  inputParameters.device = Pa_GetDefaultInputDevice(); /* default input device */
  if (inputParameters.device == paNoDevice)
  {
    fprintf(stderr, "Error: No default input device.\n");
    exit(1);
  }
  inputParameters.channelCount = 1;
  inputParameters.sampleFormat = paInt16;
  inputParameters.suggestedLatency = Pa_GetDeviceInfo(inputParameters.device)->defaultLowInputLatency;
  inputParameters.hostApiSpecificStreamInfo = NULL;

  /* Record some audio. -------------------------------------------- */
  err = Pa_OpenStream(&stream, &inputParameters, NULL, /* &outputParameters, */
                      pv_sample_rate(), paFramesPerBufferUnspecified,
                      paClipOff, /* we won't output out of range samples so don't bother clipping them */
                      NULL, NULL);
  if (err != paNoError)
    exit(1);

  err = Pa_StartStream(stream);
  if (err != paNoError)
    exit(1);
  printf("\n=== Now recording!! Please speak into the microphone. ===\n");
  fflush(stdout);

  /* -- Here's the loop where we pass data from input to output -- */
  bool is_finalized = false;
  while (data.sampleIndex < data.maxSampleIndex && !is_finalized)
  {
    {
      auto start = std::chrono::steady_clock::now();
      err = Pa_ReadStream(stream, &data.recordedSamples[data.sampleIndex], pv_rhino_frame_length());
      std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start;
      std::cout << "frame read took " << elapsed.count() << " seconds" << std::endl;
    }
    if (err)
      exit(1);

    {
      auto start = std::chrono::steady_clock::now();
      status = pv_rhino_process(rhino, &data.recordedSamples[data.sampleIndex], &is_finalized);
      std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start;
      std::cout << "detection took " << elapsed.count() << " seconds" << std::endl;
    }

    if (status != PV_STATUS_SUCCESS)
    {
      fprintf(stderr, "'pv_rhino_process' failed with '%s'\n", pv_status_to_string(status));
      exit(1);
    }

    data.sampleIndex += pv_rhino_frame_length();
  }

  bool is_understood = false;
  status = pv_rhino_is_understood(rhino, &is_understood);
  if (status != PV_STATUS_SUCCESS)
  {
    fprintf(stderr, "'pv_rhino_is_understood' failed with '%s'\n", pv_status_to_string(status));
    exit(1);
  }

  const char* intent = NULL;
  int32_t num_slots = 0;
  const char** slots = NULL;
  const char** values = NULL;

  if (is_understood)
  {
    status = pv_rhino_get_intent(rhino, &intent, &num_slots, &slots, &values);
    if (status != PV_STATUS_SUCCESS)
    {
      fprintf(stderr, "'pv_rhino_get_intent' failed with '%s'\n", pv_status_to_string(status));
      exit(1);
    }
  }

  fprintf(stdout, "{\n");
  fprintf(stdout, "    'is_understood' : '%s',\n", is_understood ? "true" : "false");
  if (is_understood)
  {
    fprintf(stdout, "    'intent' : '%s',\n", intent);
    if (num_slots > 0)
    {
      fprintf(stdout, "    'slots' : {\n");
      for (int32_t i = 0; i < num_slots; i++)
      {
        fprintf(stdout, "        '%s' : '%s',\n", slots[i], values[i]);
      }
      fprintf(stdout, "    }\n");
    }
  }
  fprintf(stdout, "}\n");

  return err;
}
