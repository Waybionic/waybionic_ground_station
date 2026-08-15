# CAN Control Architecture

## End-to-End Network Diagram
This defines the physical and logical boundaries of the Waybionic 6-DOF arm control network.

```text
[ Doctor Controller / Network ] 
         | (Ethernet/Wi-Fi)
         v
[ Robot Computer (Host) ]  ---(USB3/GigE)---> [ Cameras ]
  |-- ROS 2 High-Level 
  |-- ros2_control Hardware Interface
  |-- SocketCAN Abstraction
         | (CAN-FD)
         v
[ Logical CAN Channel (vcan0 / can0) ]
  |--> [ Joint 1 Node ]
  |--> [ Joint 2 Node ]
  |--> [ Joint 3 Node ]
  |--> [ Joint 4 Node ]
  |--> [ Joint 5 Node ]
  |--> [ Joint 6 Node ]

* Note: Power, E-Stop, Motor Enable, and Hardware Safety loops operate on a completely separate hardware layer from the CAN bus.
```

## Responsibilities
* **Host (Robot PC):** Computes kinematics, trajectories, and safety limits. Sends high-level position/velocity targets to the bus. Decodes joint feedback and publishes `sensor_msgs/msg/JointState`. Monitors CAN heartbeat/health and publishes to `/diagnostics`. **Does not generate individual step pulses.**
* **Joint Nodes (Drives):** Close the local motor control loops (PID). Convert target pos/vel into actual motor currents/steps. Broadcast current position, velocity, and health/heartbeat back to the CAN bus.

## Protocol Evaluation: `ros2_canopen` vs. Direct SocketCAN
**1. `ros2_canopen` (CiA 402)**
* **Pros:** Highly standardized. Plug-and-play if we purchase off-the-shelf (COTS) smart actuators that natively run the CANopen CiA 402 motion profile.
* **Cons:** Massive overhead. The CANopen state machine is complex, and the SDO/PDO mapping can be rigid and difficult to debug.

**2. Direct SocketCAN (Custom Protocol)**
* **Pros:** Extremely low overhead. Allows us to fully utilize CAN-FD's 64-byte payload to pack pos/vel/health into single frames. 
* **Cons:** Requires us to define our own frame IDs and data packing.

**Recommendation & Decision:**
We will proceed with **Direct SocketCAN** wrapped in a clean, hardware-independent abstraction layer. 
* *If Electrical designs custom joint-controller PCBs:* We have the lightweight protocol we need.
* *If Mechanical chooses COTS CANopen motors:* Our abstraction layer allows us to seamlessly swap the transport backend to `ros2_canopen` later without rewriting the core `ros2_control` logic.
*(Provisional 6-node IDs and data layouts will be used until hardware is finalized).*

## Useful Websites
- https://www.csselectronics.com/pages/can-fd-flexible-data-rate-intro
- https://docs.kernel.org/networking/can.html
- https://github.com/linux-can/socketcand
- https://github.com/ros-industrial/ros2_canopen
- https://docs.openarm.dev/api-reference/can/