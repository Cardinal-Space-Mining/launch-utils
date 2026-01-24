## Launch Utils
This repo provides a framework for launching large ROS2 projects dynamically and efficiently. It also contains builtin routines for automatically launching common ROS2 utilities, such as `foxglove_bridge`, `bag play`/`bag record`, `joy_node`, and `robot_state_publisher`, so they no longer need to be managed by individual packages/projects!

#### Highlighed features:
- Single-file config for an entire project
- Multiple presets per action/routine, with inheritance support to minimize code duplication
- Single-pass action "dependency" handling
- Easy remapping/reconfiguration of presets using CLI args
- Dynamic API to support highly customized routines/use cases

## How It Works
<details> <summary>Click to show!</summary>
At the heart of the utility is a configuration file defined in JSON, augmented with special pragma tags that enable preprocessing. The system is designed to be declarative, composable, and override-friendly, allowing complex runtime configurations to be expressed compactly. The general structure is as follows:

### 1. Actions
Actions are JSON objects defined inside the top-level (unnamed) JSON object. For example:
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
Each action represents a configurable process, routine, or capability that can be enabled, disabled, or parameterized. A common example would be a ROS node that can be launched with different parameters, remappings, or runtime behavior.

### 2. Presets / Basic Preprocessing
Each action contains one or more presets, which are named JSON objects describing a complete configuration for that action.

An action is configured by selecting exactly one preset, or disabled entirely by selecting no preset. A default preset may be defined using the special `pragma:default` tag, which is used when no external override is provided.

Example:
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
### 3. Preset Derivation (Inheritance)
Presets can inherit from one another using the `pragma:derived` tag. This allows new presets to be created by selectively overriding an existing base configuration instead of redefining everything.

When a preset derives from another, the following rules apply:

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
Preprocessed output when `preset1` is selected would look like:
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
To allow a single preset selection to influence multiple actions, presets may define a `pragma:action_overrides` block. This specifies additional default preset selections for other actions.

Override precedence:
1. External (CLI / API) overrides
2. `pragma:action_overrides`
3. Each action’s own `pragma:default`

Overrides are applied in a single pass. If an action has already been resolved, later overrides targeting it are ignored, preventing cyclical dependency issues.

Example:
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
        }
    },
    "action2":
    {
        "pragma:default": null,
        "preset2":
        {
            "value1": "w"
        }
    }
}
```
Resolved output:
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
Note how `action2.preset2`'s override of `action1` is ignored since `action1` has already been resolved when that block is read.

<!-- ### 5. Global Constants and Value Resolution
The configuration may define a top-level `pragma:constants` block. Constants are resolved once, globally, and may be referenced from any preset.

```json
{
    "pragma:constants":
    {
        "image_width": 640,
        "image_height": 480,
        "area": "${{ image_width * image_height }}"
    }
}
```
Constants can be used in presets via:

- Value References
    ```json
    "width": "${image_width}"
    ````

- Expressions
    ```json
    "buffer_size": "${{ area / 2 }}"
    ```

Expressions are evaluated safely and may reference:

- Constants
- Previously resolved values
- A small whitelist of math helpers (min, max, abs, math)

References and expressions may be nested arbitrarily and are resolved after preset inheritance. -->

### 5. File Imports
The special block `pragma:import` may be included, which specifies local or absolute filepaths to additional json config files. These are recursively loaded and merged with the main json blob before any preprocessing is done.

When merging, files loaded earlier always take precedence which disallows cyclical imports and means that value conflicts for a common key favor the value of first occurance.

Example:
```json
{
    // a.json

    "pragma:enable_preproc": true,
    "pragma:import": ["b.json"],

    "action1":
    {
        "pragma:default": "preset1",
        "preset1":
        {
            "value1": "x"
        },
        "preset2":
        {
            "value1": 7
        }
    }
}

{
    // b.json
    "action1":
    {
        "pragma:default": "preset2",
        "preset1":
        {
            "value1": "y",
            "value2": "z"
        },
        "preset3":
        {
            "value1": 8
        }
    },
    "action2":
    {
        "pragma:default": "preset1",
        "preset1":
        {
            "value1": "w"
        }
    }
}
```
Result:
```json
{
    "action1":
    {
        "value1": "x",
        "value2": "z"
    },
    "action2":
    {
        "value1": "w"
    }
}
```

### 6. Additional Config Details
1. Preprocessing only occurs if `"pragma:enable_preproc": true` is set at the top level. This prevents accidental multiple preprocessing passes.
2. Keys set to null at any level are pruned from the final result.
3. Assigning an empty object {} overrides a dictionary without removing it.
4. Assigning a partial dictionary merges recursively.
5. The optional `pragma:node_options` block may be provided inside a preset to configure ROS2 Node launch options. All entries are passed as keyword arguments, except `"remappings"`, which are converted into a list of tuples.

    Example debug preset:
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

### 7. Launch Arguments
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
