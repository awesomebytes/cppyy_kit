#!/usr/bin/env bash
_depth_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
_depth_root=$(cd -- "$_depth_dir/../../../../.." && pwd)
export PYTHONPATH="$_depth_root:$_depth_dir:$_depth_root/build/perception_toolbox/repos/Depth-Anything-3/src${PYTHONPATH:+:$PYTHONPATH}"
export HF_HOME="$_depth_root/build/perception_toolbox/huggingface"
unset _depth_dir _depth_root
