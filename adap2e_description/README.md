# adap2e_description

## 1) Overview

`adap2e_description` provides the robot-specific description layer for the Adap2e mobile bases.

It extends `romea_mobile_base_description` with the concrete configuration, URDF/Xacro files, meshes and `ros2_control` descriptions required to instantiate Adap2e robots.

The supported Adap2e variants are:

| Variant | Configuration file | Mobile base architecture |
|---|---|---|
| `one` | `config/adap2e_one.yaml` | `4WS4WD` |
| `two` | `config/adap2e_two.yaml` | `4WS4WD` |

Both variants are four-wheel steering and four-wheel drive mobile bases. They share the same description structure, but use different geometric parameters.

## 2) Robot configuration

The `config/` directory contains the robot configuration files used by the Python API and by the Xacro descriptions.

Each Adap2e configuration follows the structure defined by `romea_mobile_base_description` and contains:

* the mobile base architecture (`4WS4WD`);
* geometry, wheel dimensions and chassis bounding box;
* wheel steering and wheel speed command limits;
* sensor feedback characteristics;
* inertia and control point;
* link and joint names used in the URDF and `ros2_control` descriptions.

The package also provides `config/teleop.yaml`, the default teleoperation configuration used by `adap2e_bringup`.

## 3) URDF and ros2_control descriptions

The URDF description is built from:

| Path | Role |
|---|---|
| `urdf/adap2e_one.urdf.xacro` | entry point for the `one` variant |
| `urdf/adap2e_two.urdf.xacro` | entry point for the `two` variant |
| `urdf/adap2e.xacro` | common Adap2e mobile base macro |
| `urdf/adap2e.simulation.xacro` | simulator-specific Gazebo or Gazebo Classic plugin insertion |
| `meshes/` | chassis and wheel visual meshes |

The common Adap2e macro reuses the `base4WS4WD.chassis.xacro` template from `romea_mobile_base_description` and specializes it with Adap2e geometry, link names, joint names and visual meshes.

The `ros2_control` description is built from:

| Path | Role |
|---|---|
| `ros2_control/adap2e_one.ros2_control.urdf.xacro` | entry point for the `one` variant |
| `ros2_control/adap2e_two.ros2_control.urdf.xacro` | entry point for the `two` variant |
| `ros2_control/adap2e.ros2_control.xacro` | common Adap2e `ros2_control` macro |

Depending on the selected mode, the `ros2_control` description selects:

| Mode | Hardware plugin |
|---|---|
| `live` | `adap2e_hardware/Adap2eHardware` |
| `simulation_gazebo` | `romea_mobile_base_gazebo/GazeboSystemInterface4WS4WD` |
| `simulation_gazebo_classic` | `romea_mobile_base_gazebo/GazeboSystemInterface4WS4WD` |
| `simulation_4dv`, `simulation_isaac` | `romea_mobile_base_simulation/GenericSimulationSystemInterface` |

## 4) Python API

The installed Python module provides helper functions used by `adap2e_bringup` and by the meta-bringup workflow.

| Function | Purpose |
|---|---|
| `get_specifications_path_file(robot_model)` | returns the configuration file path for `one` or `two` |
| `get_specifications_configuration(robot_model)` | loads the full robot configuration |
| `get_configuration(robot_model)` | returns the compact mobile base configuration completed with manufacturer, model and version |
| `generate_configuration_file(configuration, extended)` | serializes the compact configuration |
| `generate_urdf_description(...)` | generates the Adap2e URDF description |
| `generate_ros2_control_description(...)` | generates the Adap2e `ros2_control` description |

Example:

```python
from adap2e_description import get_configuration

configuration = get_configuration("one")
```

## 5) Relation with other packages

`adap2e_description` is the Adap2e specialization of `romea_mobile_base_description`:

* `romea_mobile_base_description` provides the generic `4WS4WD` chassis and `ros2_control` templates;
* `adap2e_description` provides the Adap2e configuration, meshes and Xacro specialization;
* `adap2e_hardware` provides the live `ros2_control` hardware plugin;
* `adap2e_bringup` uses this package to generate configuration, URDF and `ros2_control` artifacts for live and simulation modes.
