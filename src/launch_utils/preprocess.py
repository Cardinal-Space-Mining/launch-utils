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
import ast
import math
import re
# import traceback

# from collections import defaultdict, deque


# =========================
# Pragma tags
# =========================

MARKER_TAG = 'pragma:enable_preproc'
DEFAULT_TAG = 'pragma:default'
DERIVED_TAG = 'pragma:derived'
# CONSTANTS_TAG = 'pragma:constants'
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
# # Value resolution (refs + expressions)
# # =========================

# # Regex for references and expressions
# REF_RE = re.compile(r'^\$\{\s*([^}]+?)\s*\}$')
# EXPR_RE = re.compile(r'^\$\{\{\s*(.+?)\s*\}\}$')

# # Allowed functions for safe_eval
# ALLOWED_GLOBALS = {
#     "math": math,
#     "min": min,
#     "max": max,
#     "abs": abs,
# }

# class DAGResolverError(Exception):
#     pass

# def safe_eval(expr: str, context: dict):
#     """Safely evaluate a restricted Python expression."""
#     tree = ast.parse(expr, mode='eval')
#     for node in ast.walk(tree):
#         if not isinstance(node, (
#             ast.Expression, ast.BinOp, ast.UnaryOp, ast.Num, ast.Constant,
#             ast.Name, ast.Load, ast.Call, ast.Attribute,
#             ast.Add, ast.Sub, ast.Mult, ast.Div, ast.Pow, ast.USub
#         )):
#             raise ValueError(f"Disallowed expression: {expr}")

#     return eval(
#         compile(tree, "<expr>", "eval"),
#         {"__builtins__": {}},
#         {**ALLOWED_GLOBALS, **context}
#     )

# def walk_leaves(data, prefix=()):
#     """Yield all leaf paths and values in a nested dict/list structure."""
#     if isinstance(data, dict):
#         for k, v in data.items():
#             yield from walk_leaves(v, prefix + (k,))
#     elif isinstance(data, list):
#         for i, v in enumerate(data):
#             yield from walk_leaves(v, prefix + (i,))
#     else:
#         yield prefix, data

# def get_by_path(data, path):
#     cur = data
#     for p in path:
#         cur = cur[p]
#     return cur

# def set_by_path(data, path, value):
#     cur = data
#     for p in path[:-1]:
#         cur = cur[p]
#     cur[path[-1]] = value

# def resolve_relative_path(name: str, base_path: tuple, data: dict, constants: dict):
#     """
#     Resolve a reference name relative to a nested path.
#     - Fully qualified names (with '.') are returned as-is
#     - Unqualified names climb the parent chain
#     - Checks the constants block if not found in the config
#     """
#     parts = tuple(name.split('.'))
#     if len(parts) > 1:
#         return parts  # fully qualified

#     # climb parent scopes
#     for i in range(len(base_path), -1, -1):
#         candidate = base_path[:i] + parts
#         try:
#             get_by_path(data, candidate)
#             return candidate
#         except KeyError:
#             continue

#     # check constants at top-level
#     if name in constants:
#         return (name,)

#     raise KeyError(f"Cannot resolve relative reference '{name}' from {base_path}")

# def extract_dependencies(value: str, base_path: tuple, data: dict, constants: dict):
#     deps = set()

#     # simple references ${var}
#     for ref in REF_RE.findall(value):
#         dep_path = resolve_relative_path(ref.strip(), base_path, data, constants)
#         deps.add(dep_path)

#     # expressions ${{ expr }}
#     for expr in EXPR_RE.findall(value):
#         tree = ast.parse(expr, mode="eval")
#         for node in ast.walk(tree):
#             if isinstance(node, ast.Name):
#                 dep_path = resolve_relative_path(node.id, base_path, data, constants)
#                 deps.add(dep_path)
#             elif isinstance(node, ast.Attribute):
#                 parts = []
#                 n = node
#                 while isinstance(n, ast.Attribute):
#                     parts.append(n.attr)
#                     n = n.value
#                 if isinstance(n, ast.Name):
#                     parts.append(n.id)
#                     dep_path = resolve_relative_path(".".join(reversed(parts)), base_path, data, constants)
#                     deps.add(dep_path)
#     return deps

# def build_dependency_graph(data: dict, constants: dict):
#     """
#     Build DAG of dependencies: {node_path: set(dep_paths)}
#     Only includes nodes that are references or expressions.
#     """
#     graph = defaultdict(set)
#     nodes = {}

#     for path, value in walk_leaves(data):
#         if not isinstance(value, str):
#             continue
#         if not (REF_RE.search(value) or EXPR_RE.search(value)):
#             continue
#         nodes[path] = value
#         base_path = path[:-1]
#         deps = extract_dependencies(value, base_path, data, constants)
#         graph[path].update(deps)

#     return graph, nodes

# def topo_sort(graph):
#     """
#     Perform topological sort on the dependency graph.
#     Raises DAGResolverError if a cycle is detected.
#     """
#     in_degree = defaultdict(int)
#     for node, deps in graph.items():
#         for dep in deps:
#             in_degree[dep] += 1

#     queue = deque(node for node in graph if in_degree[node] == 0)
#     resolved = []

#     while queue:
#         node = queue.popleft()
#         resolved.append(node)
#         for n in graph:
#             if node in graph[n]:
#                 in_degree[n] -= 1
#                 if in_degree[n] == 0:
#                     queue.append(n)

#     if len(resolved) != len(graph):
#         raise DAGResolverError("Circular or unresolvable dependencies detected")
#     return resolved

# def resolve_all_values_dag(data: dict, constants: dict):
#     """
#     Resolve all string references and expressions in nested dict/list using DAG.
#     Returns a resolved copy of data.
#     """
#     data = copy.deepcopy(data)
#     constants = copy.deepcopy(constants)

#     graph, nodes = build_dependency_graph(data, constants)
#     if not graph:
#         return data  # nothing to resolve

#     sorted_nodes = topo_sort(graph)

#     # context combines constants + current resolved data
#     context = {**constants, **data}

#     for path in sorted_nodes:
#         value = nodes[path]
#         # evaluate expressions
#         m_expr = EXPR_RE.match(value)
#         m_ref = REF_RE.match(value)

#         if m_expr:
#             expr = m_expr.group(1)
#             resolved = safe_eval(expr, context)
#         elif m_ref:
#             ref_path = resolve_relative_path(m_ref.group(1).strip(), path[:-1], data, constants)
#             resolved = get_by_path(context, ref_path)
#         else:
#             raise DAGResolverError(f"Unresolvable value at {path}: {value}")

#         set_by_path(data, path, resolved)
#         # update context for subsequent evaluations
#         cur = context
#         for p in path[:-1]:
#             if p not in cur:
#                 cur[p] = {}
#             cur = cur[p]
#         cur[path[-1]] = resolved

#     return data


# =========================
# Main entry point
# =========================

def preprocess_launch_json(config: dict, overrides: dict = None) -> dict:
    """
    Preprocess the launch JSON config.
    """
    if not config.get(MARKER_TAG, False):
        return config

    config = copy.deepcopy(config)
    config.pop(MARKER_TAG, None)

    overrides = copy.deepcopy(overrides) if overrides else {}
    result = {}

    # constants = config.pop(CONSTANTS_TAG, {})

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
            # resolved = resolve_all_values_dag(resolved, constants)

            result[action] = prune_nulls(resolved)
            print(f"[LAUNCH PREPROC]: Configured action '{action}' with preset '{chosen_preset}'")
        else:
            print(f"[LAUNCH PREPROC]: Removed action '{action}'")

    return result
