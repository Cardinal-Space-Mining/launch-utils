## Launch Utils
This repo provides a framework for launching large ROS2 projects dynamically and efficiently. It also contains builtin routines for automatically launching common ROS2 utilities, such as `foxglove_bridge`, `bag play`/`bag record`, `joy_node`, and `robot_state_publisher`, so they no longer need to be managed by individual packages/projects!

#### Highlighed features:
- Single-file config for an entire project
- Multiple presets per action/routine, with inheritance support to minimize code duplication
- Single-pass action "dependency" handling
- Easy remapping/reconfiguration of presets using CLI args
- Dynamic API to support highly customized routines/use cases

## How It Works
<details>
<summary>Click to show!</summary>
At the heart of the utility is a configuration file, which is defined in JSON with special marker tags which allow for preprocessing. The general structure goes as follows:

### 1. Actions
Actions are simply JSON objects defined inside the highest level unnamed JSON object. For example:
```json
{
    "pragma:enable_preproc": true,
    "action1":
    {
        ...
    },
    "action2":
    {
        ...
    }
}
```
Generally speaking, an action represents a process/routine/action that can occur, which either requires a configuration to run or can be enabled/disabled. An obvious example of this would be a ROS node, which can be run and configured with params, remappings, etc.

### 2. Presets/Basic Preprocessing
Inside each action is a variable number of JSON objects which represent different configuration presets. The idea here is that a given action can be configured with any of it's predefined presets, or **disabled by configuring it with no preset**. A default preset can be defined using the special `pragma:default` tag, which is used by the preprocessor in case no external mapping is defined. Consider the following example:
```json
{
    "pragma:enable_preproc": true,
    "action1":
    {
        "pragma:default": "preset1",
        "preset1":
        {
            "value1": true
        },
        "preset2":
        {
            "value1": 67
        }
    },
    "action2":
    {
        "pragma:default": null,
        "preset1":
        {
            "value1": "hi mom"
        }
    }
}
```
Here, `action1` has two presets defined and by default will use `preset1`. `action2` has a single preset defined but will be disabled unless explicitly configured otherwise. Note that any action who's preset mapping evaluates to `null` (pure JSON), `None` (python), or the string representation of either will be disabled by the preprocessor.

The primary objective of the preprocessor is to map each action to a single preset, or remove it in the case that it is assigned no preset. For the previous code block, the preprocessed output would be as follows:
```json
{
    "action1":
    {
        "value1": true
    }
}
```
Note how `action1` obtained the contents of `preset1` (it's default), and `action2` was removed entirely since it's default preset was set to `null`.

### 3. Preset Derivation
Presets are very powerful on their own, but get clunky when you have to redefine every single value within them for each individual preset. This is where inheritance comes in. A preset can be "derived" from another preset using the `pragma:derived` tag, in which case the preprocessor implements the following behavior:
- The parent preset is **fully copied.**
- The child preset is **merged into the parent recursively.**
- Values in the child override the parent's values.
- If the child sets a key to `null`, that key is **removed** from the final result.
- Dot-notation keys (ex. `"foo.bar": 1`) expand into nested overrides.

Consider the following:
```json
{
    "pragma:enable_preproc": true,
    "action1":
    {
        "pragma:default": "preset1",
        "preset1":
        {
            "crop_box":
            {
                "min": [1, 1, 1],
                "max": [2, 2, 2]
            },
            "filters":
            {
                "pass": true,
                "noise": 0.1
            }
        },
        "preset2":
        {
            "pragma:derived": "preset1",
            "crop_box.min": [0, 0, 0],
            "filters.noise": 0.05,
            "frame_ids":
            {
                "base_frame": "robot",
                "odom_frame": "odom"
            }
        },
        "preset3":
        {
            "pragma:derived": "preset2",
            "filters": null,
            "frame_ids":
            {
                "map_frame": "map"
            }
        }
    }
}
```
Preprocessed output when `preset1`is selected would look like:
```json
{
    "action1":
    {
        "crop_box":
        {
            "min": [1, 1, 1],
            "max": [2, 2, 2]
        },
        "filters":
        {
            "pass": true,
            "noise": 0.1
        }
    }
}
```
Likewise, for `preset2`:
```json
{
    "action1":
    {
        "crop_box":
        {
            "min": [0, 0, 0],
            "max": [2, 2, 2]
        },
        "filters":
        {
            "pass": true,
            "noise": 0.05
        },
        "frame_ids":
        {
            "base_frame": "robot",
            "odom_frame": "odom"
        }
    }
}
```
And finally, `preset3`:
```json
{
    "action1":
    {
        "crop_box":
        {
            "min": [0, 0, 0],
            "max": [2, 2, 2]
        },
        "frame_ids":
        {
            "base_frame": "robot",
            "odom_frame": "odom",
            "map_frame": "map"
        }
    }
}
```

### 4. Action Dependencies
To allow multiple actions to be reconfigured using a single preset override, the preprocessor also handles basic dependency management. Each config may define a `pragma:action_overrides` block which specifies an additional layer of default presets for other actions. These overrides have higher priority than each action's singly defined default, but lower priority than externally specified overrides. The preprocessor also handles these in a single pass, so actions that are resolved first will override later ones, and won't be configured according to later actions' overrides if a cyclical reference is introduced. Consider the following:
```json
{
    "pragma:enable_preproc": true,
    "action1":
    {
        "pragma:default": "preset1",
        "preset1":
        {
            "pragma:action_overrides":
            {
                "action2": "preset2"
            },
            "value1": "x"
        },
        "preset2":
        {
            "value1": "y"
        }
    },
    "action2":
    {
        "pragma:default": null,
        "preset1":
        {
            "value1": "z"
        },
        "preset2":
        {
            "pragma:action_overrides":
            {
                "action1": "preset1"
            },
            "value1": "w"
        }
    }
}
```
Which would be resolved as:
```json
{
    "action1":
    {
        "value1": "x"
    },
    "action2":
    {
        "value1": "w"
    }
}
```
Note how `action2.preset2`'s override of `action1` are ignored since `action1` has already been resolved when that block is read.

### 5. Additional Config Details
Along with the above preprocessing behavior, the following is also important to note:
1. No preprocessing will occur unless the `"pragma:enable_preproc": true` assignment if found in the base scope (alongside the actions). This is to guard against situations where multiple preprocessing stages exist due to package usage constraints, and the config should only be preprocessed once.
2. The `pragma:node_options` block may be provided inside an action preset to further control the ROS2 launch `Node` object (python). All assignments are read/passed as kwargs except `"remappings"`, which are converted to a list of tuples for convenience. This block only gets used if the provided `NodeAction` class (see [actions.py](src/launch_utils/actions.py)) is used to extract and format the launch object using an action config. As an example, if you want to make a preset to debug a C++ node, you could do:
```json
"debug":
{
    "pragma:derived": "presetX",
    "pragma:node_options":
    {
        "prefix": ["xterm -e gdb -ex run --args"]
    }
}
```

### 6. Launch Args
A config file alone will only ever be preprocessed a single way, so we use additional arguments to reassign presets to each action. Using the python API, args are passed as a dict of `action : preset` key-value pairs, which get passed to the preprocessor when resolving a config file. Remapping can additionally be exposed to the CLI using the provided parser, which takes launch args in the format `key:=value` and exports a dictionary, which is then already in the correct format for the preprocessor to use.

The best way to see this in action is to take a look at the [example launchfile](launch/test.launch.py), although if correctly implemented, allows for the following usage:
```bash
ros2 launch test.launch.py action1:=preset2 action2:=null ... actionX:=presetY
```
This allows support for multiple completely different launch setups to be spawned using the same config - controlled by only a few CLI args!

</details>

## Usage
See the example [launchfile](launch/test.launch.py) for common python API usage and the example [config file](config/test.json) for example preprocessor usage.

The following action blocks are predefined and can be resolved automatically by the python API:
- `foxglove_bridge` : creates a foxglove_bridge node
- `foxglove_gui` : launches foxglove studio application
- `joy_node` : creates a joy node
- `robot_tf` : creates a robot_state_publisher node (uses custom JSON --> URDF converter/spec - see [tf_converter.py](src/launch_utils/tf_converter.py) for implementation, as I am too lazy to write the docs for this at the moment)
- `bag` : creates a ros2 bag player (requires bag:=BAGPATH launch arg)
- `bag_record` : creates a ros2 bag recorder
- `bag_rerecord` : creates a ros2 bag recorder configured for re-recording a bag which is being played

Implementation details can be found in [actions.py](src/launch_utils/actions.py) and usage examples in [test.json](config/test.json)
