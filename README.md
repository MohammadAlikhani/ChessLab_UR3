# ROS Chess-Playing Robot Project

## Introduction
This project, developed by Marco Alikhani Najafabadi implements a robotic system using the **Robot Operating System (ROS)** to play chess on a physical board. The system integrates chessboard state detection, and robotic manipulation to detect chess pieces, understand the board state, and execute physical moves using a robotic arm (UR3 with a Robotiq-85 gripper).

## Project Goals
The primary objectives of the system are:
1. **Board Detection**: Use a camera to detect ArUco markers on chess pieces.
2. **Board State Understanding**: Identify which piece occupies each square on the chessboard.
3. **Move Decision**: Interface with an external chess engine to decide moves (not implemented in the ROS nodes).
4. **Move Execution**: Physically pick up and place chess pieces using a robotic arm.

## System Overview
The project is divided into **two main stages**:
1. **Chess Board Mapping**: Detects ArUco markers, maps them to chess pieces and squares, and saves the board state to a YAML file.
2. **Trajectory Execution**: Orchestrates robotic arm movements to execute chess moves based on the board state and target positions.

### Components
- **Camera and ArUco Markers**:
  - The `aruco_marker_publisher` node publishes marker data to the `/aruco_marker_publisher/markers` topic.
  - ArUco markers are attached to chess pieces, each with a unique ID (e.g., 315 for "kingW").
- **Chess Board Mapper**:
  - Subscribes to `/aruco_marker_publisher/markers` for marker ID and 3D pose data.
  - Converts marker IDs to piece names (e.g., 315 → "kingW") and physical (x, y) coordinates to chess notation (e.g., (0.175, -0.175) → "H8").
  - Outputs a YAML file (e.g., `kingW: "e1"`) with the current board state, updated with each marker detection.
- **ArUco Pose Listener Node**:
  - Listens to `/aruco_marker_publisher/markers` for `aruco_msgs::msg::MarkerArray` messages.
  - Extracts marker ID, 3D position, orientation, and confidence, then writes this data to a YAML file (e.g., `aruco_poses.yaml`), configurable via a ROS parameter.
- **ChessMapper Library** (`ChessMapper.cpp`):
  - **PieceToMarker**: Maps chess piece names (e.g., "pawnB1") to marker IDs (e.g., 201) using a `std::map`. Returns -1 if the piece is not found.
  - **MarkerToPiece**: Reverse mapping from marker IDs to piece names (e.g., 201 → "pawnB1"). Returns an empty string if the ID is not found.
  - **PositionToSquare**: Converts physical (x, y) coordinates to chess notation (e.g., "A1", "H8") based on a board layout with 0.05-unit square size.
  - **SquareToPosition**: Converts chess notation to physical (x, y) coordinates in the UR3 base frame using an affine transformation (rotation, translation).
- **Trajectory Execution**:
  - Managed by the `piece_to_ik.cpp` node, which orchestrates the sequence of actions.
  - Uses external ROS 2 services and actions:
    - `/get_marker_Tf`: Retrieves ArUco marker pose in the base frame.
    - `/inverse_kinematics`: Computes multiple inverse kinematics (IK) solutions.
    - `/joint_trajectory_controller/follow_joint_trajectory`: Controls UR3 arm movements.
    - `/GripperClose` and `/GripperOpen`: Controls the Robotiq-85 gripper.

### Key Challenges
- **Coordinate Mapping**: Accurately mapping chess notation to physical board coordinates (cm-precision).
- **Frame Transformation**: Converting camera/world frames to the UR3 base frame using TF (Transform).
- **Inverse Kinematics (IK)**: Generating reliable IK solutions despite arm redundancies.
- **Smooth Trajectories**: Ensuring safe and smooth robotic arm movements.

### Workflow
1. **Initialization**:
   - Parse command-line arguments for piece (e.g., "queenW") and target square (e.g., "e4").
   - Use `PieceToMarker` to get the ArUco ID and `SquareToPosition` to compute target coordinates.
   - Initialize ROS clients, action client, and TF listener.
2. **Grasp Pose Calculation**:
   - Map piece name to ArUco ID using `PieceToMarker`.
   - Query `/get_marker_Tf` for the marker’s transform (base → aruco_<id>).
   - Adjust translation for grasp point (+15 cm in Z for marker height, +10 cm for pre-grasp pose).
   - Convert quaternion to UR3 end-effector convention.
3. **Inverse Kinematics**:
   - Send each pose (pre-grasp, grasp, place-hover, place-down) to `/inverse_kinematics`.
   - Select the closest joint solution using Euclidean norm in joint space (`pick_closest`).
   - Store solutions for pre_ik_, grasp_ik_, place_hover_ik_, and place_down_ik_.
4. **Trajectory Execution**:
   - Populate `JointTrajectoryPoint` arrays with varying durations (2s, 1.5s, 3s) for smooth motion.
   - Chain actions using result callbacks (e.g., pre-hover → grasp → gripper close → retract).
   - Use timers (e.g., 3s) to sequence actions like gripper operations and retractions.

## Repository Structure
The repository contains the ROS nodes, libraries, and scripts for the chess-playing robot. Key files include:
- `ChessMapper.cpp`: Core library for piece/marker and position/square conversions.
- `piece_to_ik.cpp`: Main node for orchestrating the move sequence.
- YAML output files (e.g., `aruco_poses.yaml`, board state YAML) for storing marker and board data.


[aruco_broadcaster](https://drive.google.com/file/d/1537hm3XQs6YSUC_E06nzRHWWZKO6eu-X/view?usp=sharing)

[Project Video](https://drive.google.com/file/d/1cb08yu_hgBxhSI6n_dHF99slnW_3weM7/view?usp=sharing)

[Presentation Slides](https://drive.google.com/file/d/1WSWgCSHS3WsIzgwHeuwGZK6BWWuKif3o/view?usp=sharing)


## Getting Started
1. **Clone the Repository**:

2. **Build the packages**:
   - Build the aruco_broadcaster: `colcon build --packages-select aruco_broadcaster`
   - Source the workspace: `source install/setup.bash`
   - Build rest of the packages: `colcon build`  
  
3. **Run launch file and Node**:
   - Prepare the enviroment: `ros2 launch chesslab_setup2 combined_aruco_listener.launch.py` 
   - Launch the `piece_to_ik` node (e.g., `ros2 run chesslab_setup2 piece_to_ik queenB e4`).
   - Before runinng this node is `piece_to_ik` better to put object in the middle to see better results
4. **Dependencies**:
   - ROS 2 services: `/get_marker_Tf`, `/inverse_kinematics`, `/joint_trajectory_controller/follow_joint_trajectory`, `/GripperClose`, `/GripperOpen`.


## Future Improvements
- Enhance robustness of IK solutions for complex board positions.
- Optimize trajectory durations for faster moves.


## Acknowledgments
Thank you to the Professor rosell for his support and guidance in building this project.
