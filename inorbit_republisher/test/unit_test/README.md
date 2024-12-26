# Unit Test for ROS Republisher Node

This repository contains a unit test for a ROS node that republishes messages from `/input_topic` to `/output_topic`. The test ensures the republisher node processes and republishes messages correctly.

## Test Description

The unit test is implemented in `test_republisher.py` and uses the `unittest` framework along with `rostest`. It includes the following test cases:

- **`test_message_republishing`**: 
  - Publishes a test message (`key=value`) to `/input_topic`.
  - Verifies that the republisher node republishes the message with the correct prefix (`input_to_output=`) to `/output_topic`.

- **`test_no_message`**:
  - Ensures that no messages are received on `/output_topic` at the start of the test.

The test subscribes to `/output_topic` and collects received messages for validation.

## Running the Unit Test

1. **Build the Workspace**:
   Ensure the workspace is built and the environment is sourced:
   ```bash
   cd ~/catkin_ws
   rosdep install --from-paths ~/catkin_ws/src --ignore-src --rosditro=noetic
   catkin clean 
   catkin_build inorbit_republisher --verbose
2. **Source and Run the Workspace**:
    ```bash
    . ~/catkin_ws/devel/setup.bash
	rostest inorbit_republisher  test_republisher.test
    ```