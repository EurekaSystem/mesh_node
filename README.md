# Mesh Skill 

## Overview

The **mesh** package implements a ROS 2 skill using an action-based interface.
It is designed as a modular component that can be triggered via an action client and extended with custom logic.

---

## Requirements

* ROS 2 
* Python 3
* `colcon` build system

### Ros dependencies

All required ROS dependencies are declared in the two `package.xml` files.

### Python dependencies

| Package | Version |
|---|---|
| `open3d` | `0.19.0` | 
| `numpy` | `1.26.4` | 
---

## Build the Package

Source your ROS 2 environment and build the workspace:

```bash
source /opt/ros/<your ros2 version>/setup.bash
cd <your_workspace>
colcon build
source install/setup.bash
```

---

## Run the Skill

Launch the skill using:

```bash
ros2 launch mesh mesh.launch.py
```

Expected Output:

```text
Initialising...
Skill mesh started, but not yet configured.
Skill mesh configuration started.
Skill mesh is configured, but not yet active.
Skill mesh is active and running.
Skill mesh: performing background task...
```

---

## Architecture

The skill is implemented as:

* A ROS 2 node (lifecycle-based)
* An action server (default name: `mesh_reference`)
* A background execution loop

Main logic is located in:

```
mesh/skill_impl.py
```

---

## Mesh Reconstruction Requirements

### Parameters

The following parameters are part of the **custom action interface** `mesh_skill_msgs/action/Mesh`:

- **scan_id** (`string`)  
  Uniquely identifies the object for which the mesh should be generated.


- **result** (`int64`)  
  Represents the execution result, allowing different success or failure states.
To ensure the mesh reconstruction pipeline runs smoothly, make sure the following conditions are satisfied.

---

### Input Data Structure

The skill expects the following directory structure:

```
<DATA_DIR>/
└── <scan_id>/
    └── pcl.ply
```

* `<DATA_DIR>`: base directory defined in your code
* `<scan_id>`: unique identifier passed to the skill
* `pcl.ply`: input point cloud file

Example:

```
data/
└── 001/
    └── pcl.ply
```

---

### Output Files

The skill will generate the following files inside the same folder:

```
<DATA_DIR>/<scan_id>/
├── mesh_<scan_id>.ply
└── mesh_<scan_id>_double.ply
```

* `mesh_<scan_id>.ply`: reconstructed mesh
* `mesh_<scan_id>_double.ply`: double-sided mesh

---

## Testing the skill

You can test the skill using a simple ROS 2 action client.

### Example Action Client

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from mesh_skill_msgs.action import Mesh


class Act_cli(Node):

    def __init__(self):
        super().__init__('act_cli')
        self._action_client = ActionClient(self, Mesh, 'mesh_reference')

    def send_goal(self):
        goal_msg = Mesh.Goal()
        goal_msg.scan_id = "<your id>"

        # Waiting server
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Server not ready!')
            return

        # Send goal
        self.get_logger().info('Goal sent')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, self._send_goal_future)  
        self.goal_response_callback(self._send_goal_future)




    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: '+ str(result.result))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = Act_cli()
    action_client.send_goal()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

Expected Output:

```text
Goal sent
Goal accepted

```

---

## Important Notes

* The action name (`mesh_reference`) must match between:

  * Action server (in `skill_impl.py`)
  * Action client

* After modifying messages (`mesh_skill_msgs`), rebuild the workspace:

  ```bash
  colcon build
  source install/setup.bash
  ```

---

## Customization

### Modify Skill Logic

Edit:

```
mesh/skill_impl.py
```

This file controls:

* Goal handling
* Execution logic
* Returned results

---

### Modify Action Interface

Edit the action definition:

```
mesh_skill_msgs/action/Mesh.action
```

You can customize:

* Goal fields (input)
* Result fields (output)
* Feedback messages

---

### Change Action Name

If needed, update the action name:

* In the server (`skill_impl.py`)
* In the client (`mesh_reference` → your custom name)

---

## Common failure cases


