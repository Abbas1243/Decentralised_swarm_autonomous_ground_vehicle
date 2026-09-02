#!/bin/bash
# spawn_swarm.sh
# Usage: ./spawn_swarm.sh <num_robots>
# Example: ./spawn_swarm.sh 4
#
# This script:
#   1. Spawns N turtles in TurtleSim (turtle2, turtle3, ...)
#   2. Launches one robot_agent_node per turtle
#
# Run TurtleSim first in a separate terminal:
#   ros2 run turtlesim turtlesim_node

NUM_ROBOTS=${1:-3}   # default 3 robots if no argument given

echo "Spawning $NUM_ROBOTS robot turtles..."

# Spread initial spawn positions so turtles don't overlap
for i in $(seq 0 $((NUM_ROBOTS - 1))); do
    TURTLE_IDX=$((i + 2))            # turtle2, turtle3, ...
    SPAWN_X=$(echo "2 + $i * 1.5" | bc -l)
    SPAWN_Y="3.0"
    THETA="0.0"
    TURTLE_NAME="turtle${TURTLE_IDX}"

    echo "  Spawning $TURTLE_NAME at ($SPAWN_X, $SPAWN_Y)"

    ros2 service call /spawn turtlesim/srv/Spawn \
        "{x: ${SPAWN_X}, y: ${SPAWN_Y}, theta: ${THETA}, name: '${TURTLE_NAME}'}" \
        > /dev/null 2>&1

    sleep 0.3
done

echo "Turtles spawned. Launching robot agents..."

# Launch one agent node per robot in background
PIDS=()
for i in $(seq 0 $((NUM_ROBOTS - 1))); do
    TURTLE_IDX=$((i + 2))
    NODE_NAME="robot_agent_${i}"

    ros2 run swarm_positioning robot_agent_node \
        --ros-args \
        -p robot_id:=$i \
        -p num_robots:=$NUM_ROBOTS \
        -r __node__:=$NODE_NAME &

    PIDS+=($!)
    echo "  Started $NODE_NAME (PID $!)"
    sleep 0.2
done

echo ""
echo "All $NUM_ROBOTS robot agents running."
echo "Now launch the human tracker in another terminal:"
echo "  ros2 run swarm_positioning human_tracker_node"
echo ""
echo "Press Ctrl+C here to stop all robot agents."

# Wait and cleanup on exit
trap "echo 'Stopping...'; kill ${PIDS[@]} 2>/dev/null" SIGINT SIGTERM
wait