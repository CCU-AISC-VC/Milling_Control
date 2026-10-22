#!/usr/bin/env bash
# Re-run the four demo scenes at a slower time-lapse and render their RTX close-ups.
# Usage: scripts/render_all_demos.sh [speed=5] [subframes=8]
set -eo pipefail
SPEED=${1:-5}; SUB=${2:-8}
PY=~/envs/aisc_sim/bin/python
cd "$(dirname "$0")/.."
export OMNI_KIT_ACCEPT_EULA=YES
run() { echo "=== $(date +%H:%M:%S) $*"; "$@"; }
# 1) single pocket/contour (M3)
run $PY scripts/m3_sim_replay.py --traj outputs/m2_ik/sample_pocket_contour/trajectory --video-speed $SPEED > outputs/m3_sim/replay_single.log 2>&1
run $PY scripts/m3_render_isaacsim.py --usd outputs/m3_sim/sample_pocket_contour/scene.usd --closeup --subframes $SUB --orbit-deg 30 > outputs/m3_sim/sample_pocket_contour/rtx_closeup.log 2>&1
echo STEP1_DONE
# 2) dual cell (M6)
run $PY scripts/m6_dual_replay.py --traj fanuc_1=outputs/m6_dual/traj_fanuc_1/trajectory --traj fanuc_2=outputs/m6_dual/traj_fanuc_2/trajectory --video-speed $SPEED > outputs/m6_dual/replay.log 2>&1
run $PY scripts/m3_render_isaacsim.py --usd outputs/m6_dual/scene.usd --cell configs/cells/dual_robot_table.yaml --camera closeup --subframes $SUB --orbit-deg 25 > outputs/m6_dual/rtx_closeup.log 2>&1
echo STEP2_DONE
# 3) 5-axis wave
run $PY scripts/m3_sim_replay.py --traj outputs/m2_ik/sample_5axis_wave/trajectory --video-speed $SPEED > outputs/m3_sim/replay_5axis.log 2>&1
run $PY scripts/m3_render_isaacsim.py --usd outputs/m3_sim/sample_5axis_wave/scene.usd --closeup --subframes $SUB --orbit-deg 30 > outputs/m3_sim/sample_5axis_wave/rtx_closeup.log 2>&1
echo STEP3_DONE
# 4) CCU emblem (long program: 3x the speed of the others)
run $PY scripts/m3_sim_replay.py --traj outputs/m2_ik/ccu_emblem/trajectory --video-speed $((SPEED*3)) > outputs/m3_sim/replay_emblem.log 2>&1
run $PY scripts/m3_render_isaacsim.py --usd outputs/m3_sim/ccu_emblem/scene.usd --closeup --subframes $SUB --orbit-deg 30 > outputs/m3_sim/ccu_emblem/rtx_closeup.log 2>&1
echo STEP4_DONE
echo ALL_DONE
