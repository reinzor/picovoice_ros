/*
    Copyright 2018-2020 Picovoice Inc.

    You may not use this file except in compliance with the license. A copy of the license is located in the "LICENSE"
    file accompanying this source.

    Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on
    an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the
    specific language governing permissions and limitations under the License.
*/

#include <alsa/asoundlib.h>
#include <dlfcn.h>
#include <stdio.h>

#include "pv_rhino.h"

int main(int argc, char* argv[])
{
  if (argc != 4)
  {
    fprintf(stderr, "usage : %s model_path context_path input_audio_device\n", argv[0]);
    exit(1);
  }

  const char* model_path = argv[1];
  const char* context_path = argv[2];
  const char* input_audio_device = argv[3];

  pv_rhino_t* rhino = NULL;
  pv_status_t status = pv_rhino_init(model_path, context_path, 0.5f, &rhino);
  if (status != PV_STATUS_SUCCESS)
  {
    fprintf(stderr, "'pv_rhino_init' failed with '%s'\n", pv_status_to_string(status));
    exit(1);
  }

  snd_pcm_t* alsa_handle = NULL;
  int error_code = snd_pcm_open(&alsa_handle, input_audio_device, SND_PCM_STREAM_CAPTURE, 0);
  if (error_code != 0)
  {
    fprintf(stderr, "'snd_pcm_open' failed with '%s'\n", snd_strerror(error_code));
    exit(1);
  }

  snd_pcm_hw_params_t* hardware_params = NULL;
  error_code = snd_pcm_hw_params_malloc(&hardware_params);
  if (error_code != 0)
  {
    fprintf(stderr, "'snd_pcm_hw_params_malloc' failed with '%s'\n", snd_strerror(error_code));
    exit(1);
  }

  error_code = snd_pcm_hw_params_any(alsa_handle, hardware_params);
  if (error_code != 0)
  {
    fprintf(stderr, "'snd_pcm_hw_params_any' failed with '%s'\n", snd_strerror(error_code));
    exit(1);
  }

  error_code = snd_pcm_hw_params_set_access(alsa_handle, hardware_params, SND_PCM_ACCESS_RW_INTERLEAVED);
  if (error_code != 0)
  {
    fprintf(stderr, "'snd_pcm_hw_params_set_access' failed with '%s'\n", snd_strerror(error_code));
    exit(1);
  }

  error_code = snd_pcm_hw_params_set_format(alsa_handle, hardware_params, SND_PCM_FORMAT_S16_LE);
  if (error_code != 0)
  {
    fprintf(stderr, "'snd_pcm_hw_params_set_format' failed with '%s'\n", snd_strerror(error_code));
    exit(1);
  }

  error_code = snd_pcm_hw_params_set_rate(alsa_handle, hardware_params, pv_sample_rate(), 0);
  if (error_code != 0)
  {
    fprintf(stderr, "'snd_pcm_hw_params_set_rate' failed with '%s'\n", snd_strerror(error_code));
    exit(1);
  }

  error_code = snd_pcm_hw_params_set_channels(alsa_handle, hardware_params, 1);
  if (error_code != 0)
  {
    fprintf(stderr, "'snd_pcm_hw_params_set_channels' failed with '%s'\n", snd_strerror(error_code));
    exit(1);
  }

  error_code = snd_pcm_hw_params(alsa_handle, hardware_params);
  if (error_code != 0)
  {
    fprintf(stderr, "'snd_pcm_hw_params' failed with '%s'\n", snd_strerror(error_code));
    exit(1);
  }

  snd_pcm_hw_params_free(hardware_params);

  error_code = snd_pcm_prepare(alsa_handle);
  if (error_code != 0)
  {
    fprintf(stderr, "'snd_pcm_prepare' failed with '%s'\n", snd_strerror(error_code));
    exit(1);
  }

  const int32_t frame_length = pv_rhino_frame_length();

  int16_t* pcm = static_cast<int16_t*>(malloc(sizeof(int16_t) * frame_length));
  if (!pcm)
  {
    printf("failed to allocate memory for audio buffer\n");
    return 1;
  }

  const char* context_info = NULL;
  status = pv_rhino_context_info(rhino, &context_info);
  if (status != PV_STATUS_SUCCESS)
  {
    fprintf(stderr, "'pv_rhino_context_info' failed with '%s'\n", pv_status_to_string(status));
    exit(1);
  }
  fprintf(stdout, "%s\n\n", context_info);

  bool is_finalized = false;

  while (!is_finalized)
  {
    const int count = snd_pcm_readi(alsa_handle, pcm, frame_length);
    if (count < 0)
    {
      fprintf(stderr, "'snd_pcm_readi' failed with '%s'\n", snd_strerror(count));
      exit(1);
    }
    else if (count != frame_length)
    {
      fprintf(stderr, "read %d frames instead of %d\n", count, frame_length);
      exit(1);
    }

    status = pv_rhino_process(rhino, pcm, &is_finalized);
    if (status != PV_STATUS_SUCCESS)
    {
      fprintf(stderr, "'pv_rhino_process' failed with '%s'\n", pv_status_to_string(status));
      exit(1);
    }
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

  if (is_understood)
  {
    status = pv_rhino_free_slots_and_values(rhino, slots, values);
    if (status != PV_STATUS_SUCCESS)
    {
      fprintf(stderr, "'pv_rhino_free_slots_and_values' failed with '%s'\n", pv_status_to_string(status));
      exit(1);
    }
  }

  status = pv_rhino_reset(rhino);
  if (status != PV_STATUS_SUCCESS)
  {
    fprintf(stderr, "'pv_rhino_reset' failed with '%s'\n", pv_status_to_string(status));
    exit(1);
  }

  free(pcm);
  snd_pcm_close(alsa_handle);
  pv_rhino_delete(rhino);

  return 0;
}
