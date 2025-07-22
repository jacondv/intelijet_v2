# BLK ARC Module-API Changelog

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html)

## [v0.9.0]

### Added

- Example script to interact with the Wifi Client
- Example script to get network information

## [v0.8.0]

### Added

- Example script to record scan-data-stream
- Example script to show how to accumulate the pointcloud
- Implemented call and examples for lidar and slam camera masking
- Documentation about the BLK ARC device and API
- Renaming and restructuring of code-base

### Changed

- Improved ROS Wrapper test robustness

## [v0.7.0]

### Added

- Implemented call to mark static poses while scanning
- ROS support for triggering detail images and starting/ending static poses

## [v0.6.0]

### Added

- Implemented call to trigger detail images

## [v0.5.0]

### Added

- Implemented call to get device status
- Implemented call to get free disk space information
- Implemented call to get the firmware version
- Script to generate blk_arc_api Python package
- Add reboot service to ROS wrapper

### Changed

- Split up the blk_arc_example.py into multiple examples
- Update API to use secure gRPC channel

## [v0.4.0]

### Added

- Restart functionality with example
- Added status to determine if the device is in USB-only mode
- Clock sources and timesync example

### Fixed

- Fixes in requirements and pytests, additionally suggesting using a virtual environment in readme

## [v0.3.0]

### Added

- ROS support in the form of the catkin packages ros_blkarc, ros_blkarc_driver and ros_blkarc_msgs

### Changed

- Revised requirements.txt to specify version number for required Python modules

## [v0.2.1]

### Fixed

- Return value of is_connected()-method of BLK_ARC-class now returns a bool as intended

## [v0.2.0]

### Added

- List the scans on the scanner
- List metadata of scans on the scanner
- Delete scans on the scanner
- Download scans from the scanner

## [v0.1.0]

### Added

- Proto-Definitions of the relevant GRPC-Calls to communicate with BLK ARC-devices
- Script to generate python code from the proto-definitions
- Generated python code
- Connection-configuration for the BLK ARC
- Sample BLK_ARC-class to show how to use the generated API-calls
- Example script to show how to use the BLK_ARC-class
- Basic readme explaining how to setup everything and requirements
- This Changelog
- Pre-Commit Hooks to ensure code quality
- Pytests to test API and as example usage
