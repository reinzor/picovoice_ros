# picovoice_driver

ROS Wrappers for the [Picovoice](https://picovoice.ai/) libraries.

## Nodes

### picovoice_driver_porcupine

ROS interface for the [Porcupine wake word engine](https://picovoice.ai/platform/porcupine/).

_More documentation about the recognizer can be found [here](https://github.com/Picovoice/porcupine)_

```
rosrun picovoice_driver picovoice_driver_porcupine
```

#### Parameters

- `~model_url` (default=`package://picovoice_driver/extern/picovoice/resources/models/porcupine_params.pv`): Path to the Porcupine Picovoice model parameters. This URL should start with `package://` or `file://`.
- `~keywords_directory_url` (default=`package://picovoice_driver/extern/picovoice/resources/keywords`): URL to the keywords directory. Keywords in this directory will be found if the keyword url of the goal does not start with `package://` or `file://`. This URL should start with `package://` or `file://`.
- `~record_timeout` (default=`300`): An incoming action goal will be aborted when this timeout is exceeded and nothing was recognized.
- `~record_directory` (default=`/tmp/picovoice_driver/porcupine`): Record audio samples will be stored in this directory. If you do not want to store the audio samples, this parameter can be set to `''`.
- `~sensitivity` (default=`0.5`): Sensitivity of the recognizer. See the Picovoice docs for more details.

#### ROS action interfaces

- `get_keyword` ([picovoice_msgs/GetKeyword](../picovoice_msgs/action/GetKeyword.action)): Get a keyword of a user voice command by specifying a list of keyword candidates

### picovoice_driver_rhino

ROS interface for the [Rhino speech to intent engine](https://picovoice.ai/platform/rhino/).

_More documentation about the recognizer can be found [here](https://github.com/Picovoice/rhino)_

```
rosrun picovoice_driver picovoice_driver_rhino
```

#### Parameters

- `~model_url` (default=`package://picovoice_driver/extern/picovoice/resources/models/porcupine_params.pv`): Path to the Rhino Picovoice model parameters. This URL should start with `package://` or `file://`.
- `~contexts_directory_url` (default=`package://picovoice_driver/extern/picovoice/resources/contexts`): URL to the contexts directory. Contexts in this directory will be found if the context url of the goal does not start with `package://` or `file://`. This URL should start with `package://` or `file://`.
- `~record_timeout` (default=`300`): An incoming action goal will be aborted when this timeout is exceeded and nothing was recognized.
- `~record_directory` (default=`/tmp/picovoice_driver/porcupine`): Record audio samples will be stored in this directory. If you do not want to store the audio samples, this parameter can be set to `''`.
- `~sensitivity` (default=`0.5`): Sensitivity of the recognizer. See the Picovoice docs for more details.

#### ROS action interfaces

- `get_intent` ([picovoice_msgs/GetIntent](../picovoice_msgs/action/GetIntent.action)): Get an intent of a user voice command by specifying a context
