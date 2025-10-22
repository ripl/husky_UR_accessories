# Robotiq S-Model Control

This package provides ROS control interfaces for the Robotiq S-Model three-finger adaptive gripper. The S-Model gripper features fingers A and B on one side (similar to index and middle fingers) and finger C on the opposite side (similar to a thumb), enabling versatile grasping modes.

## Controllers

### SModelSimpleController.py
Basic command-line interface for simple gripper control with unified finger movement.

**Usage:**
```bash
rosrun robotiq_s_model_control SModelSimpleController.py
```

**Features:**
- Basic activation/reset
- Four grasping modes (Basic, Pinch, Wide, Scissor)
- Unified position, speed, and force control
- Simple keyboard interface

### SModelIndependentController.py (Enhanced)
Advanced command-line interface with individual finger control capabilities.

**Usage:**
```bash
rosrun robotiq_s_model_control SModelIndependentController.py
```

**Key Features:**
- **All Simple Controller functions preserved**
- **Individual Control of Fingers (rICF)**: Independent control of fingers A, B, and C
- **Individual Control of Scissor (rICS)**: Independent control of the scissor axis
- **Per-finger position, speed, and force control**
- **Mode switching between unified and independent control**

## Control Modes

### Grasping Modes
- **Basic Mode (0)**: Standard grasp with moderate finger spacing
- **Pinch Mode (1)**: Fingers A and B close together for precision grasping
- **Wide Mode (2)**: Fingers A and B spread wide for larger objects
- **Scissor Mode (3)**: A and B fingers move along scissor axis to clamp objects

### Independent Control
When **Individual Finger Control (rICF=1)** is enabled:
- Each finger (A, B, C) can be controlled independently
- Separate position, speed, and force settings per finger
- Allows complex grasping strategies

When **Individual Scissor Control (rICS=1)** is enabled:
- Scissor axis moves independently from grasping mode
- Grasping mode (rMOD) is ignored
- Scissor position controlled by rPRS register

## Independent Controller Commands

### Basic Control
- `r`: Reset gripper
- `a`: Activate gripper  
- `c`: Close (unified mode)
- `o`: Open (unified mode)

### Grasping Modes
- `b`: Basic mode
- `p`: Pinch mode
- `w`: Wide mode
- `s`: Scissor mode

### Individual Control
- `I`: Toggle Individual Finger Control (rICF)
- `S`: Toggle Scissor Control (rICS)
- `1`: Select Finger A
- `2`: Select Finger B  
- `3`: Select Finger C
- `4`: Select Scissor axis

### Position Control
- `0`, `5-255`: Set position for unified mode or current finger (1-4 reserved for finger selection)
- `+`: Increase position by 10
- `-`: Decrease position by 10
- `C`: Close current finger/axis fully
- `O`: Open current finger/axis fully

### Speed & Force Control  
- `f`: Increase speed
- `l`: Decrease speed
- `i`: Increase force
- `d`: Decrease force

## Message Interface

Both controllers publish to the `SModelRobotOutput` topic using the `robotiq_s_model_articulated_msgs/SModelRobotOutput` message type.

**Key message fields:**
- `rICF`: Individual Control of Fingers (0=unified, 1=individual)
- `rICS`: Individual Control of Scissor (0=normal, 1=independent)
- `rPRA/rPRB/rPRC`: Position for fingers A/B/C (0-255)
- `rSPA/rSPB/rSPC`: Speed for fingers A/B/C (0-255)
- `rFRA/rFRB/rFRC`: Force for fingers A/B/C (0-255)
- `rPRS/rSPS/rFRS`: Scissor position/speed/force (0-255)

## Usage Examples

### Basic Grasping Workflow
1. Start the controller: `rosrun robotiq_s_model_control SModelIndependentController.py`
2. Activate gripper: `a`
3. Set grasping mode: `p` (pinch mode)
4. Close gripper: `c`

### Independent Finger Control Workflow
1. Enable individual control: `I`
2. Select finger A: `1`
3. Set finger A position: `200`
4. Select finger B: `2` 
5. Set finger B position: `150`
6. Select finger C: `3`
7. Close finger C: `C`

### Scissor Control Workflow
1. Enable scissor control: `S`
2. Select scissor axis: `4`
3. Set scissor position: `100`

## Launch Parameters

Both controllers accept the following ROS parameters:
- `~topic` (default: `/UR_1/SModelRobotOutput`): Output topic name

## Dependencies

- `robotiq_s_model_articulated_msgs`
- Standard ROS packages (`rospy`, `roslib`)

## Maintainer

- Original Simple Controller: Robotiq, Inc.
- Enhanced Independent Controller: Extended for individual finger control

For detailed technical documentation, refer to the Robotiq S-Model manual at support.robotiq.com. 