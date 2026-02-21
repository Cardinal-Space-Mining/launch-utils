'''
/*******************************************************************************
*   Copyright (C) 2025-2026 Cardinal Space Mining Club                         *
*                                                                              *
*                                 ;xxxxxxx:                                    *
*                                ;$$$$$$$$$       ...::..                      *
*                                $$$$$$$$$$x   .:::::::::::..                  *
*                             x$$$$$$$$$$$$$$::::::::::::::::.                 *
*                         :$$$$$&X;      .xX:::::::::::::.::...                *
*                 .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :               *
*                :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.              *
*               :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.              *
*              ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::               *
*               X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.               *
*                .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                *
*                 X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                  *
*                 $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                    *
*                 $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                    *
*                 $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;                    *
*                 X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                     *
*                 $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                    *
*               x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                   *
*              +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                  *
*               +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                   *
*                :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$                    *
*                ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                     *
*               ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                            *
*               ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                               *
*               :;;;;;;;;;;;;.  :$$$$$$$$$$X                                   *
*                .;;;;;;;;:;;    +$$$$$$$$$                                    *
*                  .;;;;;;.       X$$$$$$$:                                    *
*                                                                              *
*   Unless required by applicable law or agreed to in writing, software        *
*   distributed under the License is distributed on an "AS IS" BASIS,          *
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
*   See the License for the specific language governing permissions and        *
*   limitations under the License.                                             *
*                                                                              *
*******************************************************************************/
'''

import copy
import re
import time


# =========================
# Pragma tags
# =========================

MARKER_TAG = 'pragma:enable_preproc'
DEFAULT_TAG = 'pragma:default'
DERIVED_TAG = 'pragma:derived'
CONSTANTS_TAG = 'pragma:constants'
PRESET_OVERRIDE_TAG = 'pragma:action_overrides'


# =========================
# String helpers
# =========================

def does_eval_null(val):
    if not isinstance(val, str):
        return False
    low = val.lower()
    return low in ('none', 'null', 'false')


def does_eval_default(val):
    if not isinstance(val, str):
        return False
    low = val.lower()
    return low in ('default', DEFAULT_TAG.lower())


# =========================
# Dot-key expansion
# =========================

def expand_dot_keys(d: dict) -> dict:
    """
    Expand keys containing '.' into nested dictionaries.
    Example: {"a.b.c": 1} -> {"a": {"b": {"c": 1}}}
    """
    result = {}
    for k, v in d.items():
        if "." in k:
            parts = k.split(".")
            current = result
            for part in parts[:-1]:
                current = current.setdefault(part, {})
            current[parts[-1]] = v
        else:
            result[k] = v
    return result


# =========================
# Deep merge
# =========================

def deep_merge(base: dict, override: dict) -> dict:
    """
    Recursively merge override into base.
    - Dicts merge deeply
    - None shadows/removes keys
    - Dot-notation keys are expanded
    """
    override = expand_dot_keys(override)
    result = copy.deepcopy(base)

    for k, v in override.items():
        if v is None:
            result.pop(k, None)
        elif isinstance(v, dict) and isinstance(result.get(k), dict):
            result[k] = deep_merge(result[k], v)
        else:
            result[k] = copy.deepcopy(v)

    return result


# =========================
# Null pruning
# =========================

def prune_nulls(data):
    """
    Recursively remove keys with None values.
    """
    if isinstance(data, dict):
        return {k: prune_nulls(v) for k, v in data.items() if v is not None}
    elif isinstance(data, list):
        return [prune_nulls(v) for v in data]
    else:
        return data


# =========================
# Preset resolution
# =========================

def resolve_preset(block: dict, preset_name: str, seen=None) -> dict | None:
    """
    Resolve a preset with inheritance (pragma:derived).
    Detects circular inheritance.
    """
    if preset_name is None:
        return None

    if preset_name not in block:
        raise ValueError(f"Preset '{preset_name}' not found in block {list(block.keys())}")

    seen = seen or set()
    if preset_name in seen:
        raise ValueError(f"Circular inheritance detected at preset '{preset_name}'")
    seen.add(preset_name)

    preset = copy.deepcopy(block[preset_name])

    if DERIVED_TAG in preset:
        parent_name = preset.pop(DERIVED_TAG)
        parent_config = resolve_preset(block, parent_name, seen)
        if parent_config is None:
            raise ValueError(f"Parent preset '{parent_name}' for '{preset_name}' is null")
        merged = deep_merge(parent_config, preset)
    else:
        merged = preset

    return prune_nulls(merged)


# =========================
# Linked action overrides
# =========================

def extract_linked_overrides(resolved_block: dict, overrides: dict) -> None:
    """
    Appends linked action preset overrides and removes the block.
    Only adds overrides that are not already set.
    """
    if PRESET_OVERRIDE_TAG in resolved_block:
        links = resolved_block.pop(PRESET_OVERRIDE_TAG)
        for k, v in links.items():
            if k not in overrides:
                overrides[k] = v
                print(f"[LAUNCH PREPROC]: Added preset override '{v}' for action '{k}'")


# # =========================
# # Value resolution
# # =========================

REF_RE = re.compile(r'^\$\{\s*([^}]+?)\s*\}$')

def resolve_constant(link: str, constants: dict):
    parts = link.split('.')
    scope = constants
    for part in parts[:-1]:
        if isinstance(scope, dict) and part in scope:
            scope = scope[part]
        else:
            return None
    if isinstance(scope, dict) and parts[-1] in scope:
        return copy.deepcopy(scope[parts[-1]])
    else:
        return None

def resolve_constants(elem, constants: dict):
    if isinstance(elem, dict):
        for k, v in elem.items():
            if resolved := resolve_constants(v, constants):
                elem[k] = resolved
    elif isinstance(elem, list):
        for i in range(len(elem)):
            if resolved := resolve_constants(elem[i], constants):
                elem[i] = resolved
    elif isinstance(elem, str) and (match := REF_RE.search(elem)):
        return resolve_constant(match.group(1), constants)

    return None


# =========================
# Main entry point
# =========================

def preprocess_launch_json(config: dict, overrides: dict = None) -> dict:
    """
    Preprocess the launch JSON config.
    """
    start_t = time.perf_counter()

    if not config.get(MARKER_TAG, False):
        return config

    config = copy.deepcopy(config)
    config.pop(MARKER_TAG, None)
    constants = config.pop(CONSTANTS_TAG, {})
    overrides = copy.deepcopy(overrides) if overrides else {}
    result = {}
    retry_actions = {}
    for action, block in config.items():
        if not isinstance(block, dict) or action.startswith('pragma:'):
            continue

        default_preset = block.get(DEFAULT_TAG)
        chosen_preset = overrides.get(action, default_preset)

        if chosen_preset is None or does_eval_null(chosen_preset):
            chosen_preset = None
        elif does_eval_default(chosen_preset):
            chosen_preset = default_preset

        resolved = resolve_preset(block, chosen_preset)
        if resolved is not None:
            extract_linked_overrides(resolved, overrides)
            resolve_constants(resolved, constants)
            result[action] = prune_nulls(resolved)
            print(f"[LAUNCH PREPROC]: Configured action '{action}' with preset '{chosen_preset}'")
        else:
            retry_actions[action] = block

    for action, block in retry_actions.items():
        if action in overrides:
            override = overrides[action]
            if does_eval_null(override):
                override = None
            elif does_eval_default(override):
                override = block.get(DEFAULT_TAG)
            resolved = resolve_preset(block, override)
            if resolved is not None:
                resolve_constants(resolved, constants)
                result[action] = prune_nulls(resolved)
                print(f"[LAUNCH PREPROC]: Configured action '{action}' with preset '{override}'")
                continue

        print(f"[LAUNCH PREPROC]: Removed action '{action}'")

    end_t = time.perf_counter()
    print(f'[LAUNCH PREPROC]: Finished in {((end_t - start_t) * 1000):.1f} milliseconds')

    return result
