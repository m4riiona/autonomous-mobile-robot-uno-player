# Autonomous UNO-Playing Mobile Robot (ROS & YOLOv11)

An autonomous mobile robot system developed as the Final Project for the Advanced Robotics course in the Bachelor's Degree in Artificial Intelligence at the Barcelona School of Informatics (FIB - UPC).

The system integrates reactive wall-following navigation, map-based waypoint navigation, real-time computer vision detection, and an asynchronous decision-making state machine to play a simulated round of the UNO card game.

## Key Features

* **Dual-Mode Navigation:** Combines a reactive LiDAR-based wall follower (without a map) for exploration and an AMCL / `move_base` waypoint navigator for precise positioning in front of target cards.
* **Real-Time Computer Vision:** Uses a custom-trained **YOLOv11 mini** model (trained on 2,500+ images with heavy data augmentation) to detect and classify UNO cards under varying lighting conditions.
* **Centralized Orchestration:** A robust ROS actionlib-based state machine (`uno_orchestrator_ros`) that manages the mission workflow, handles game logic, applies a strict strategy hierarchy, and optimizes color changing dynamically based on statistical analysis of the robot's hand.

## Repository Structure

* `uno_wall_navigation_actualizado`: LiDAR processing, wall-following controller, and waypoint mission configuration.
* `uno_detector_ros`: Image preprocessing, format decoupling (`image_republisher`), and YOLOv11 inference node.
* `uno_orchestrator_ros`: The "brain" of the project. Contains the state machine, RANK_ALIASES processing, and the UNO game logic engine.

## System Architecture

```mermaid
graph TD
    LiDAR[\LiDAR /scan/] style LiDAR fill:#f5b041,stroke:#333,stroke-width:2px
    Camera[\Camera/] style Camera fill:#f5b041,stroke:#333,stroke-width:2px
    Robot[TurtleBot3 Robot] style Robot fill:#3498db,stroke:#333,stroke-width:2px

    WF[wall_follower_node]
    MS[map_server]
    AMCL[AMCL]
    MB[move_base]
    MC[mission_controller]
    DET[uno_detector_node]
    ORCH[uno_orchestrator_node]

    LiDAR -->|/scan| WF
    WF -->|/cmd_vel| Robot
    MS -->|/map| AMCL
    MS -->|/map| MB
    AMCL -->|/amcl_pose| MB
    DET -->|/uno_detector/detections JSON| ORCH
    ORCH -->|MoveBaseAction Goals| MB
    ORCH -->|/cmd_vel adjustments| Robot
    ORCH -->|/sound| Robot
    MC -.->|Services| ORCH

## 👥 Authors
* Sam Brumwell
* Mariona Casasnovas Simon
* Martina Hernández
* Núria López Encinas
* Veronica Oñate

*Facultat d'Informàtica de Barcelona (FIB) - Universitat Politècnica de Catalunya (UPC)*


