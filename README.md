# ChatBOAT - UVMS Simulation Environment

Stonefish + ROS2 Humble simulation of a GIRONA500 I-AUV with a 4-DOF ECA 5E manipulator and magnetic gripper, replicating the TWINBOT paper's robot morphology.

## Architecture

- **chatboat_description** - URDF/xacro for the complete UVMS (vehicle + arm + gripper)
- **chatboat_stonefish** - Stonefish scenario XML, sensor/actuator config
- **chatboat_control** - Gripper service + test commander (cube stacking demo)
- **chatboat_bringup** - Meta-launch orchestrating all packages

## Quick Start (RunPod)

Stonefish requires OpenGL 4.3+. We use [RunPod](https://www.runpod.io) with a GPU pod (~$0.32/hr for RTX A4000).

### 1. Build and push the Docker image

```bash
# Log in to Docker Hub first: docker login
./scripts/build_and_push.sh <your-dockerhub-username>
```

### 2. Create a RunPod Pod

1. Go to [runpod.io/console/pods](https://www.runpod.io/console/pods)
2. Click **+ GPU Pod**
3. Select **RTX A4000** (cheapest GPU with OpenGL 4.6)
4. Change template to **Custom**:
   - **Container Image**: `<your-dockerhub-username>/chatboat-sim:latest`
   - **Expose HTTP Ports**: `6080`
   - **Container Disk**: 20 GB
   - **Volume Disk**: 0 GB
5. Click **Deploy On-Demand**

### 3. Open the simulation

Once the pod is running, click **Connect** -> **HTTP [6080]** to open the Stonefish GUI via noVNC in your browser.

### 4. Run the cube stacking demo

SSH into the pod (Connect -> SSH), then:

```bash
ros2 launch chatboat_bringup bringup.launch.py run_demo:=true
```

## ROS2 Interfaces

### Topics published by Stonefish

| Topic | Type | Hz |
|-------|------|----|
| `/girona500/joint_states` | `sensor_msgs/JointState` | 100 |
| `/girona500/imu` | `sensor_msgs/Imu` | 100 |
| `/girona500/dvl` | `stonefish_ros2/DVL` | 10 |
| `/girona500/pressure` | `sensor_msgs/FluidPressure` | 10 |
| `/girona500/odometry` | `nav_msgs/Odometry` | 50 |
| `/girona500/ft_wrist` | `geometry_msgs/WrenchStamped` | 100 |
| `/girona500/ee_camera` | `sensor_msgs/Image` | 15 |

### Topics subscribed by Stonefish

| Topic | Type |
|-------|------|
| `/girona500/thruster_commands` | `std_msgs/Float64MultiArray` (5 elements, [-1,1]) |
| `/girona500/joint_commands` | `sensor_msgs/JointState` (4 joint positions) |

### Services

| Service | Type |
|---------|------|
| `/girona500/gripper/activate` | `std_srvs/SetBool` |

## TF Tree

```
world -> girona500/base_link -> arm_base -> arm_link1 -> arm_link2 -> arm_link3 -> arm_link4 -> gripper_tip -> ee_frame
```

## Development Workflow

1. Edit code locally on your Mac
2. Rebuild and push the image: `./scripts/build_and_push.sh <username>`
3. On RunPod, stop and restart the pod (it pulls the latest image)

For faster iteration, SSH into the pod and edit + rebuild directly:

```bash
cd /ros2_ws && colcon build && source install/setup.bash
```

## Alternative: docker-compose (generic cloud GPU)

If using a cloud VM with Docker + NVIDIA runtime instead of RunPod:

```bash
docker compose -f docker/docker-compose.yml up --build
# Then open http://<vm-ip>:6080
```
