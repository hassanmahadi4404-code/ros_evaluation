This project simulates a modular multi-mission rover system in ROS 2 Jazzy using rclpy and turtlesim.
It demonstrates how to design and integrate Science, Delivery, Equipment Servicing, and Autonomous Navigation missions using proper ROS 2 communication paradigms (topics, services, actions, parameters).

🧩 System Overview
Mission Modes

Science Mission — Turtle moves to a site, “documents” it by capturing a dummy image and GNSS coordinate.

Delivery Mission — Turtle moves between pickup and delivery points using ROS 2 Actions.

Equipment Servicing Mission — Turtle performs simulated maintenance operations on a lander.

Autonomous Navigation Mission — Turtle autonomously visits multiple waypoints (GNSS & vision targets).

🏗️ System Architecture

Each mission runs as a separate node, managed by a central Mission Manager Node.

              +----------------+
              | Mission Manager|
              +--------+-------+
                       |
         +-------------+--------------+
         |             |              |
 +-------v-----+ +-----v-------+ +----v------+
 | ScienceNode | | DeliveryNode| | Equipment |
 +-------------+ +-------------+ +-----------+
                       |
                       v
              +------------------+
              | Autonomous Nav   |
              +------------------+


All missions communicate via topics, actions, and parameters.

⚙️ ROS 2 Features Used
Feature	Used For	Example
Topic	Publishing movement commands	/turtle1/cmd_vel
Service	Mission mode switching	/set_mission_mode
Action	Delivery pick-and-place tasks	/delivery_action
Parameters	Selecting mission mode dynamically	mission_mode
LifecycleNode (optional)	Managing mission states	MissionManager lifecycle transitions
🧠 Node Descriptions
🧪 1. Science Node (science_node.py)

Moves turtle to predefined waypoints (“sites”).

Simulates camera capture (dummy_image.jpg or webcam snapshot).

Logs GNSS coordinates and stores them with the image.

Uses topics to send movement commands.

Key Concepts:

Publisher for /turtle1/cmd_vel

Service or timer for taking “pictures”

File writing for simulated GNSS logs

📦 2. Delivery Node (delivery_node.py)

Implements a ROS 2 Action Server (PickAndDeliver.action).

Turtle moves to a pickup coordinate, “picks up” the package, then delivers it.

Publishes dummy confirmation messages.

Key Concepts:

ROS 2 Action Server/Client

Action feedback and result handling

Dummy “delivery complete” log

🔧 3. Equipment Servicing Node (equipment_servicing_node.py)

Simulates maintenance on a “lander”.

Performs dummy steps:

Move to lander position.

“Inspect” components.

“Tighten bolts” or “check connections.”

Publishes and logs service status updates.

Key Concepts:

Timer-based task sequence

Publisher for simulated maintenance status

Service to trigger or stop maintenance

🤖 4. Autonomous Navigation Node (autonomous_nav_node.py)

Navigates through 4 waypoints:

2 GNSS coordinates

2 vision targets (detected via hardcoded colors or IDs)

Prints LED status:

🔴 Red = Autonomous mode

🔵 Blue = Manual mode

🟢 Green = Target reached

Key Concepts:

Publisher to control turtle

Waypoint list and navigation loop

Print-based LED simulation

🧭 5. Mission Manager Node (mission_manager.py)

Central controller.

Parameter: mission_mode (science, delivery, equipment, nav)

Dynamically activates one mission at a time using parameter callbacks.

Avoids restarting ROS nodes for mission switching.

Key Concepts:

Dynamic parameter callback

Conditional logic for mission switching

Lifecycle-like state management

🚀 Launch Process
Launch File: mission_system.launch.py

Starts all nodes together and sets initial mission mode.

ros2 launch rover_missions mission_system.launch.py mission_mode:=science


You can dynamically change mission mode without relaunching:

ros2 param set /mission_manager mission_mode delivery
ros2 param set /mission_manager mission_mode equipment
ros2 param set /mission_manager mission_mode nav

🧩 Node Graph
Node	Subscribes	Publishes	Services/Actions	Purpose
mission_manager	/mission_status	/mission_command	/set_mission_mode	Control mission mode
science_node	/mission_command	/turtle1/cmd_vel, /science_log	—	Site exploration
delivery_node	/mission_command	/turtle1/cmd_vel	/delivery_action (action)	Delivery tasks
equipment_servicing_node	/mission_command	/equipment_status	/start_service	Maintenance simulation
autonomous_nav_node	—	/turtle1/cmd_vel, /led_status	—	Navigation
🧪 How to Run
1️⃣ Build the workspace
colcon build
source install/setup.bash

2️⃣ Run Turtlesim
ros2 run turtlesim turtlesim_node

3️⃣ Run Mission System
ros2 launch rover_missions mission_system.launch.py mission_mode:=science

4️⃣ Switch missions dynamically
ros2 param set /mission_manager mission_mode delivery
ros2 param set /mission_manager mission_mode equipment
ros2 param set /mission_manager mission_mode nav

5️⃣ Observe terminal outputs

Science: Logs GNSS + image.

Delivery: Prints action progress.

Equipment: Publishes maintenance status.

Navigation: Prints LED status and waypoints reached.

📁 Folder Structure
rover_missions/
│
├── launch/
│   └── mission_system.launch.py
│
├── rover_missions/
│   ├── mission_manager.py
│   ├── science_node.py
│   ├── delivery_node.py
│   ├── equipment_servicing_node.py
│   ├── autonomous_nav_node.py
│
├── action/
│   └── PickAndDeliver.action
│
├── package.xml
└── setup.py

📸 Example Outputs

Science mission:
Image saved: site_1.jpg
GNSS log: (23.81, 90.41)

Delivery mission:
Action: Moving to pickup...
Action: Delivering...
Result: Delivery complete!

Equipment servicing:
Inspecting lander... Done
Tightening bolts... Done
Maintenance complete.

Autonomous navigation:
LED: 🔴 Autonomous mode
Waypoint 2 reached -> 🟢

