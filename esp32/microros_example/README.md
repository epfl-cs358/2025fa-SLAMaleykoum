## Installation

Tutorial: https://technologiehub.at/project-posts/micro-ros-on-esp32-tutorial/

- Install ROS2 (Humble) along with `rosdep` and `colcon`, on Ubuntu 22.04 (or a docker container).

- Install the PlatformIO Vscode extension.

- Run the following commands:

```bash
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Set up the micro-ROS workspace
cd ../microros_ws

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash

# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

or if you use zsh instead of bash:

```bash
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.zsh

# Set up the micro-ROS workspace
cd ../microros_ws

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Build micro-ROS tools and source them
colcon build
source install/local_setup.zsh

# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.zsh
```

## Run

### Without Docker

NOTE: before running any command in a new terminal, source the ROS 2 installation: `source /opt/ros/$ROS_DISTRO/setup.bash`.

Upload code to the ESP32 board, then run `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0`. You should see the following output:

```
[1744148971.623603] info     | TermiosAgentLinux.cpp | init                     | running...             | fd: 3
[1744148973.269178] info     | Root.cpp           | delete_client            | delete                 | client_key: 0x10F3AD31
[1744148973.269237] info     | SessionManager.hpp | destroy_session          | session closed         | client_key: 0x10F3AD31, address: 0
[1744148973.269262] info     | Root.cpp           | create_client            | create                 | client_key: 0x1159200A, session_id: 0x81
[1744148973.269272] info     | SessionManager.hpp | establish_session        | session established    | client_key: 0x1159200A, address: 0
[1744148973.285093] info     | ProxyClient.cpp    | create_participant       | participant created    | client_key: 0x1159200A, participant_id: 0x000(1)
[1744148973.302838] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x1159200A, topic_id: 0x000(2), participant_id: 0x000(1)
[1744148973.313618] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x1159200A, publisher_id: 0x000(3), participant_id: 0x000(1)
[1744148973.325701] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x1159200A, datawriter_id: 0x000(5), publisher_id: 0x000(3)
```

In another terminal, run:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 topic list
```

You should see the following output:

```
/micro_ros_platformio_node_publisher
/parameter_events
/rosout
```

By subscribing to the `/micro_ros_platformio_node_publisher` with the command `ros2 topic echo /micro_ros_platformio_node_publisher` topic we can see the output of the ESP32 board:

```
vince ~ $ ros2 topic echo /micro_ros_platformio_node_publisher
data: 20
---
data: 21
---
data: 22
---
data: 23
...
```

### With Docker

Run `docker run -it --rm --net=host --device=/dev/ttyUSB0 microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0`

## TODO: implement UDP communication (now it's only Serial)

Run `docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6`

https://github.com/Geibinger/micro-ROS-samples/blob/main/README.md
