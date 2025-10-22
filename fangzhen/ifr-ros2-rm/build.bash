#!/bin/bash

robot_args=""
declare -A robot_map

robot_map["--aim"]="--pEnd rm_aim_launch" 
robot_map["--sentry"]=""

other_args=()
for arg in "$@"; do
    if [[ ${robot_map[$arg]+_} ]]; then
        robot_args="${robot_map[$arg]}"
    else
        other_args+=("$arg")
    fi
done
python3 $(dirname $0)/../../tools/builder.py $0  $robot_args "${other_args[@]}" 
