#!/bin/bash

# Motor Failure Injection Script for Gazebo
# Usage: ./motor_failure.sh [-i <instance>] <motor_number>
#
# Arguments:
#   -i <instance>: Vehicle instance number (1-indexed, defaults to 1)
#   motor_number: Motor number to fail (1-indexed, 0 to clear failure)
#
# Examples:
#   ./motor_failure.sh 1           # Fail motor 1 on x500_0
#   ./motor_failure.sh 0           # Clear motor failure
#   ./motor_failure.sh -i 2 3      # Fail motor 3 on x500_1

INSTANCE=1
MOTOR_NUMBER=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -i)
            INSTANCE="$2"
            shift 2
            ;;
        *)
            MOTOR_NUMBER="$1"
            shift
            ;;
    esac
done

if [ -z "$MOTOR_NUMBER" ]; then
    echo "Error: Motor number is required"
    echo "Usage: $0 [-i <instance>] <motor_number>"
    echo ""
    echo "Arguments:"
    echo "  -i <instance>: Vehicle instance number (1-indexed, defaults to 1)"
    echo "  motor_number: Motor number to fail (1-indexed, 0 to clear failure)"
    echo ""
    echo "Examples:"
    echo "  $0 1           # Fail motor 1 on x500_0"
    echo "  $0 0           # Clear motor failure"
    echo "  $0 -i 2 3      # Fail motor 3 on x500_1"
    exit 1
fi

# Validate instance number
if [ "$INSTANCE" -lt 1 ]; then
    echo "Error: Instance number must be >= 1"
    exit 1
fi

# Construct model name and Gazebo topic name (instance is 1-indexed, model name is 0-indexed)
MODEL_INSTANCE=$((INSTANCE - 1))
MODEL_NAME="x500_${MODEL_INSTANCE}"
TOPIC="/model/${MODEL_NAME}/motor_failure/motor_number"

# Send motor failure command via Gazebo topic
if [ "$MOTOR_NUMBER" -eq 0 ]; then
    echo "Clearing motor failure on $MODEL_NAME"
else
    echo "Sending motor failure command: motor=$MOTOR_NUMBER to model=$MODEL_NAME"
fi

gz topic -t "$TOPIC" -m gz.msgs.Int32 -p "data: $MOTOR_NUMBER"

if [ $? -eq 0 ]; then
    if [ "$MOTOR_NUMBER" -eq 0 ]; then
        echo "Motor failure cleared on $MODEL_NAME"
    else
        echo "Motor $MOTOR_NUMBER failed on $MODEL_NAME"
    fi
else
    echo "Error: Failed to send motor failure command"
    echo "Make sure Gazebo is running and the model name is correct"
    exit 1
fi
