#!/usr/bin/env bash
# Collect bounded ROS 2 graph, parameter and topic reads from the active host.
set -uo pipefail

if [[ $# != 1 || -z $1 ]]; then
    printf 'Usage: %s /path/to/new-private-capture-directory\n' "$0" >&2
    exit 2
fi
if ! command -v ros2 >/dev/null || ! command -v timeout >/dev/null; then
    printf 'Source the existing ROS 2 environment and ensure GNU timeout is available.\n' >&2
    exit 2
fi
if [[ ! ${ROS_DOMAIN_ID:-} =~ ^[0-9]+$ ]]; then
    printf 'Export the ROS_DOMAIN_ID confirmed for the live processes before collecting.\n' >&2
    exit 2
fi

umask 077
out=$1
mkdir -- "$out" || exit 2
export PYTHONUNBUFFERED=1

ik=${T31_IK_NODE:-/ik_retargeting}
fk=${T31_FK_NODE:-/fk_adapter}
bridge=${T31_BRIDGE_NODE:-/exton_bridge}

capture() {
    local name=$1 seconds=$2 rc
    shift 2
    {
        printf 'utc=%s\n' "$(date -u +%FT%TZ)"
        printf 'command:'
        printf ' %q' ros2 "$@"
        printf '\n'
        timeout --signal=INT --kill-after=2s "$seconds" ros2 "$@" 2>&1
        rc=$?
        printf '\nexit=%s\n' "$rc"
    } > "$out/$name.txt" || exit 1
    printf '%s\t%s\n' "$name" "$rc" >> "$out/status.tsv" || exit 1
    return 0
}

{
    printf 'started_utc=%s\n' "$(date -u +%FT%TZ)"
    printf 'ROS_DISTRO=%s\nROS_DOMAIN_ID=%s\nROS_LOCALHOST_ONLY=%s\nRMW_IMPLEMENTATION=%s\n' \
        "${ROS_DISTRO:-unset}" "$ROS_DOMAIN_ID" "${ROS_LOCALHOST_ONLY:-unset}" \
        "${RMW_IMPLEMENTATION:-unset}"
    printf 'observed_condition=%s\n' "${T31_OBSERVATION:-unspecified}"
    printf 'ik_node=%s\nfk_node=%s\nbridge_node=%s\n' "$ik" "$fk" "$bridge"
    printf '\nCandidate processes (PID, owner, executable name and selected environment keys)\n'
    pattern='(^|[/[:space:]])(MessageBridge([.]py)?|Puppetry([.]py)?|ik_node([.]py)?|joint_to_pose_adapter([.]py)?|exton_bridge[.]py|robot_state_publisher)([[:space:]]|$)'
    while IFS= read -r pid; do
        ps -o pid=,user=,comm= -p "$pid" || continue
        if [[ ! -r /proc/$pid/environ ]]; then
            printf '  environment unavailable\n'
            continue
        fi
        while IFS= read -r -d '' entry; do
            case $entry in
                ROS_DOMAIN_ID=*|ROS_LOCALHOST_ONLY=*|RMW_IMPLEMENTATION=*) printf '  %s\n' "$entry" ;;
            esac
        done < "/proc/$pid/environ"
    done < <(pgrep -f "$pattern" || true)
} > "$out/context.txt" 2>&1 || exit 1

printf 'capture\texit_code\n' > "$out/status.tsv" || exit 1
capture nodes 8 node list --no-daemon
capture ik_node 8 node info --no-daemon "$ik"
capture fk_node 8 node info --no-daemon "$fk"
capture bridge_node 8 node info --no-daemon "$bridge"

for side in left right; do
    for field in mirror_xyz position_map orientation_map position_scale orientation_scale calib_file; do
        key="${field}_${side}"
        capture "ik_$key" 6 param get --no-daemon "$ik" "$key"
    done
done
for key in ee_position_cost ee_orientation_cost publish_commanded_ee_state \
    urdf_path root_link_name ref_link_name target_link_names use_sim_time; do
    capture "ik_$key" 6 param get --no-daemon "$ik" "$key"
done
for key in base_frame left_ee_frame right_ee_frame publish_ee_state yield_when_clutch use_sim_time; do
    capture "fk_$key" 6 param get --no-daemon "$fk" "$key"
done
for key in state_topic command_topic publish_rate_hz feedback_rate_hz use_sim_time; do
    capture "bridge_$key" 6 param get --no-daemon "$bridge" "$key"
done

topics=(
    /target_robot/left_ee/target /target_robot/right_ee/target
    /target_robot/left_ee/state /target_robot/right_ee/state
    /client_robot/joint_states /target_robot/joint_commands
)
for topic in "${topics[@]}"; do
    name=${topic#/}
    name=${name//\//_}
    capture "${name}_info" 8 topic info --no-daemon -v "$topic"
    capture "${name}_sample" 8 topic echo --once --qos-reliability best_effort "$topic"
    # hz normally runs until interrupted; the exit code alone is not a measurement.
    capture "${name}_rate" 8 topic hz -w 50 "$topic"
done
printf 'Captures saved in %s. Check status.tsv and the raw output before making a conclusion.\n' "$out"
