# adap2e

## Overview

`adap2e` groups the ROS2 packages that describe, launch and control the Adap2e mobile bases in live and simulation modes.

This repository-level README gives a map of the stack. Detailed information about each package can be found in the corresponding package README.

## Packages

| Package | Role |
| --- | --- |
| `adap2e` | Metapackage that groups the Adap2e ROS2 packages. |
| `adap2e_description` | Robot-specific description layer for the Adap2e variants, including configuration files, URDF/Xacro descriptions, meshes and ros2_control descriptions. |
| `adap2e_bringup` | Main integration entry point for generating Adap2e configuration files, URDF descriptions, ros2_control descriptions and launch files. |
| `adap2e_hardware` | Live `ros2_control` hardware plugin for Adap2e mobile bases, built on the generic `4WS4WD` hardware abstraction. |

## Usage

In most cases, start with `adap2e_bringup`. It is the user-facing entry point of the stack and the package used by `romea_mobile_base_meta_bringup` when an Adap2e model is selected from a mobile base meta-description.

The Adap2e stack is a robot-specific specialization of `romea_mobile_base`. The mobile base architecture is `4WS4WD`; `adap2e_description` provides the concrete geometry and generated descriptions, `adap2e_hardware` provides the live hardware implementation, and `adap2e_bringup` connects these pieces to the generic mobile base launch workflow.

The supported Adap2e variants are `one` and `two`.

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

The `adap2e` project was developed by Jean Laneurit in the context of the ADAP2E ANR project.

## Contact

For questions or comments about this project, please contact [Jean Laneurit](mailto:jean.laneurit@inrae.fr).
