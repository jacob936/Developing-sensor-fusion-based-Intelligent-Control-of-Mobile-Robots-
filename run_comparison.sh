# Create  script to run BOTH experiments back-to-back:
#!/bin/bash
echo "========================================"
echo "Running ASMC Experiment (3 minutes)"
echo "========================================"
# Terminal 1: Gazebo (already running)

# Terminal 2: ASMC
ros2 run thesis_control_pkg asmc_controller_node &
ASMC_PID=$!
sleep 180  # 3 minutes
kill $ASMC_PID

echo "ASMC complete. CSV saved."

echo "========================================"
echo "Running PID Experiment (3 minutes)"
echo "========================================"
# Terminal 2: PID
ros2 run thesis_control_pkg pid_controller_node &
PID_PID=$!
sleep 180  # 3 minutes
kill $PID_PID

echo "PID complete. CSV saved."
echo "========================================"
echo "Both experiments complete!"

