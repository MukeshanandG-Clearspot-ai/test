# Drone Waypoint Simulation - Documentation

## Table of Contents

1. [Introduction](#introduction)
2. [Background](#background)
3. [Nodes](#nodes)
    - [RRT](#rrt)
    - [Pilot](#pilot)
    - [Tracking](#tracking)
4. [Important Tip](#important-tip)
5. [Installation](#installation)
    - [Prerequisites](#prerequisites)
    - [Installing PX4 Firmware](#installing-px4-firmware)
    - [Installing MAVROS Package](#installing-mavros-package)
    - [Installing OpenNi](#installing-openni)
6. [Running the Simulation](#running-the-simulation)
7. [Stretch Goal](#stretch-goal)
8. [KMZ Structure and Waypoint Editing Script](#kmz-structure-and-waypoint-editing-script)
9. [Simulating with Gazebo](#simulating-with-gazebo)
10. [Relevant APIs in DJI MSDKv5](#relevant-apis-in-dji-msdkv5)


## 1. Introduction <a name="introduction"></a>

The Drone Waypoint Simulation aims to simulate the behavior of an Iris quadcopter using the PX4 firmware within the Gazebo world environment. The primary objective is to achieve 3D motion planning, where the quadcopter navigates through obstacles to reach a specified destination point. Additionally, the quadcopter can be controlled using hand gestures, providing an intuitive interface for navigation in complex aerial spaces.

## 2. Background <a name="background"></a>

### PX4
PX4 is an open-source autopilot control platform used in this project. It facilitates communication between the ROS environment and the Gazebo simulator through the MAVROS package. PX4 supports both Software-in-the-Loop (SITL) and Hardware-in-the-Loop (HITL) simulations, enabling realistic testing of flight control algorithms.

### MAVROS
MAVROS is a ROS package that provides communication drivers for various autopilots using the MAVLink communication protocol. In this project, MAVROS enables communication between the PX4 flight stack and the ROS-enabled companion computer, allowing for seamless integration of autopilot functionalities within the ROS environment.

### ROS with Gazebo
ROS integration with Gazebo enables realistic simulation of robotic systems. In this project, ROS communicates with the PX4 flight stack to receive sensor data from the simulated world and send motor and actuator commands. This integration facilitates the development and testing of navigation algorithms in a simulated environment.

## 3. Nodes <a name="nodes"></a>

### RRT <a name="rrt"></a>

The RRT node (`rrt.py`) generates a 3D obstacle-free path for the quadcopter using the Rapidly-exploring Random Tree (RRT) algorithm. It takes a map containing obstacles and a destination point as input and outputs an optimized path that avoids collisions. The generated path is then smoothed to reduce the number of waypoints, ensuring smooth and efficient navigation.

The rrt.py script is a critical component of the project, serving as the backbone of the motion planning system for the quadcopter simulation. Implemented using the Rapidly-exploring Random Tree (RRT) algorithm, this script enables the generation of obstacle-free paths in complex 3D environments. By representing the configuration space as a tree structure and iteratively sampling random points within the space, rrt.py efficiently explores the environment while avoiding collisions with obstacles. Its functionality includes tree expansion towards the goal position, collision checking to ensure safety, and path extraction from the generated tree. These capabilities not only enable autonomous navigation of the quadcopter but also ensure the safety and robustness of its flight trajectory. Ultimately, rrt.py facilitates mission planning by providing optimized paths for the quadcopter to follow, thereby enhancing the overall autonomy and reliability of the simulation.

### Pilot <a name="pilot"></a>

The Pilot node (`setMode.py`) controls the flying mode of the quadcopter. It provides functionalities to arm, disarm, takeoff, land, and navigate the quadcopter autonomously along predefined paths. Additionally, it enables control of the quadcopter using hand gestures, allowing for intuitive navigation in complex environments.

The setMode.py script is pivotal in controlling the behavior and flight modes of the quadcopter within the simulation environment. Its multifaceted functionality encompasses arming, disarming, takeoff, landing, and autonomous path following, offering comprehensive command capabilities to direct the quadcopter's actions. Leveraging the ROS framework and MAVROS package, setMode.py establishes communication with the PX4 autopilot system, enabling seamless interaction between the simulation and the flight control software. By interfacing with MAVLink messages, it orchestrates the transition between different flight modes, ensuring smooth and coordinated operations. Additionally, setMode.py integrates with other components of the project, such as the RRT-based path planner and hand-gesture tracking system, facilitating diverse modes of control and navigation. Through its robust functionality and versatility, setMode.py plays a central role in orchestrating the behavior of the quadcopter within the simulation environment, enabling a wide range of mission scenarios and flight maneuvers to be executed with precision and efficiency.

### Tracking <a name="tracking"></a>

The Tracking node (`detectHand.py`) integrates computer vision with the PX4 autopilot to enable control of the quadcopter using hand gestures. It utilizes an RGBD camera and OpenCV to track hand movements and detect fingertip positions. The detected trajectory is then synchronized with the PX4 flight stack, enabling the quadcopter to follow the trajectory in real-time.

The detectHand.py script is instrumental in enabling human-computer interaction for controlling the quadcopter's movement through hand gestures within the simulation environment. Leveraging computer vision techniques and the RGBD camera feed from devices like the ASUS Xtion Pro, detectHand.py employs image processing algorithms to track the user's hand movements in real-time. By detecting the position of the user's hand and analyzing gestures, such as finger pointing or hand waving, detectHand.py translates these movements into meaningful commands for controlling the quadcopter's trajectory. The script utilizes OpenCV for image processing tasks, extracting features from the RGBD camera feed and identifying relevant hand gestures. Through seamless integration with the ROS ecosystem, detectHand.py interfaces with the PX4 autopilot system via MAVROS, enabling the translation of detected hand gestures into corresponding commands for the quadcopter's flight control. By providing an intuitive and interactive means of controlling the quadcopter, detectHand.py enhances the user experience and opens up new avenues for human-robot interaction within the simulation environment, enabling users to command the quadcopter's movement with natural hand gestures.

## 4. Important Tip <a name="important-tip"></a>

### Offboard Mode Setup

Ensure that the flight controller receives a stream of setpoint messages before switching to offboard mode. Failure to do so may result in the rejection of offboard mode by the system. It is recommended to maintain a continuous stream of setpoint messages with a timeout of 0.5 seconds before switching to offboard mode.

## 5. Installation <a name="installation"></a>

### Prerequisites <a name="prerequisites"></a>

Before installing the simulation environment, ensure that ROS (Indigo, Kinetic, Lunar, or Melodic) and Gazebo (version 8 or later) are installed on the system. Additionally, a hardware device such as an RGBD camera (e.g., ASUS Xtion Pro) is required for hand gesture control.

### Installing PX4 Firmware <a name="installing-px4-firmware"></a>

1. Clone the PX4 Firmware repository from GitHub.
2. Build the firmware platform using the provided Makefile.
3. Source the ROS workspace and launch the PX4 SITL simulation.

### Installing MAVROS Package <a name="installing-mavros-package"></a>

1. Install MAVLink and MAVROS dependencies using `rosinstall_generator`.
2. Create a ROS workspace, merge the MAVROS repository, and update dependencies.
3. Install GeographicLib datasets and build the MAVROS package using `catkin build`.

### Installing OpenNi <a name="installing-openni"></a>

1. Install the OpenNI package for ROS using the provided launch files.
2. Launch OpenNI to start capturing RGB and depth images from the camera.
3. Visualize the camera data in RViz to ensure proper functioning.

## 6. Running the Simulation <a name="running-the-simulation"></a>

To execute the simulation, follow these steps:

1. Start the ROS core (`roscore`).
2. Launch the PX4 SITL simulation using `roslaunch px4 mavros_posix_sitl.launch`.
3. For autonomous path planning, run `map.py` to generate a path using the RRT algorithm, and then execute `setMode.py` to navigate the quadcopter along the path.
4. For hand-gesture tracking, launch the OpenNI package, run `detectRGBD.py` to track hand movements, and then execute `setMode.py` to control the quadcopter using hand gestures.

## 7. Stretch Goal <a name="stretch-goal"></a>

The stretch goal of this project is to deploy the simulation environment to a real quadcopter for testing and validation. This step involves integrating the developed algorithms with a physical quadcopter and conducting experiments to assess performance and robustness.

## 8. KMZ Structure and Waypoint Editing Script <a name="kmz-structure-and-waypoint-editing-script"></a>

## KMZ Structure

### Overview

KMZ (Keyhole Markup Language Zipped) is a compressed version of KML (Keyhole Markup Language), an XML-based file format used to display geographic data in applications like Google Earth.

### Structure

A KMZ file contains at least two components:
1. **KML File**: Main file containing geographic data in XML format.
2. **Supporting Files**: Images, textures, models, and other resources referenced in the KML file.

### Creating a KMZ File

1. **Create KML Content**: Construct KML content using XML formatting.
2. **Compress**: Compress KML file along with supporting files into a ZIP archive.
3. **Rename**: Rename ZIP archive with a .kmz extension.

### Example Code to Create a KMZ File

```python
# Python script to create a KMZ file with a waypoint

import zipfile
import xml.etree.ElementTree as ET

def create_kmz_with_waypoint(latitude, longitude, altitude, filename='waypoint.kmz'):
    # Create KML structure
    kml = ET.Element('kml', xmlns="http://www.opengis.net/kml/2.2")
    document = ET.SubElement(kml, 'Document')
    placemark = ET.SubElement(document, 'Placemark')
    point = ET.SubElement(placemark, 'Point')
    coordinates = ET.SubElement(point, 'coordinates')
    coordinates.text = f'{longitude},{latitude},{altitude}'

    # Convert KML structure to string
    kml_str = ET.tostring(kml, encoding='utf-8', method='xml')

    # Write KML string to KMZ file
    with zipfile.ZipFile(filename, 'w') as kmz:
        kmz.writestr('doc.kml', kml_str)

# Example usage:
create_kmz_with_waypoint(37.42228990140251, -122.0822035425683, 0)
```

KMZ (Keyhole Markup Zip) is a zipped file format used for storing geographic data in Google Earth and other geobrowsers. It contains one or more files, typically in KML (Keyhole Markup Language) format, along with any images or additional resources referenced by the KML files. The structure of a KMZ file typically includes:

- **doc.kml**: This file contains the main KML markup defining the geographic features to be displayed.
- **images/**: This directory may contain images referenced in the KML

# Waypoint Editing Script Documentation

## Overview

The Waypoint Editing Script allows users to dynamically add, remove, or modify waypoints in an existing KMZ file. Users can specify latitude, longitude, and altitude for each waypoint.

## Functionality

1. **Add Waypoint**: Adds a new waypoint to the KMZ file.

## Usage

### Adding a Waypoint

```python
import zipfile
import xml.etree.ElementTree as ET

def add_waypoint_to_kmz(latitude, longitude, altitude, kmz_filename):
    with zipfile.ZipFile(kmz_filename, 'a') as kmz:
        # Open KML file from KMZ
        with kmz.open('doc.kml') as kml_file:
            kml_str = kml_file.read().decode('utf-8')
            kml = ET.fromstring(kml_str)

            # Add new waypoint
            placemark = ET.SubElement(kml.find('{http://www.opengis.net/kml/2.2}Document'), 'Placemark')
            point = ET.SubElement(placemark, 'Point')
            coordinates = ET.SubElement(point, 'coordinates')
            coordinates.text = f'{longitude},{latitude},{altitude}'

        # Write updated KML back to KMZ
        kmz.writestr('doc.kml', ET.tostring(kml, encoding='utf-8', method='xml'))

# Example usage:
add_waypoint_to_kmz(37.42228990240252, -122.0822035425684, 10, 'waypoint.kmz')
```

## 9. Simulating with Gazebo <a name="simulating-with-gazebo"></a>

### Running Gazebo Simulation

### Overview

To execute the program, following the steps below:
```bash
roscore
roslaunch px4 mavros_posix_sitl.launch

# For autonomous path planning
rosrun iris_sim map.py # Generate path using RRT
rosrun iris_sim setMode.py
# First set Mode 1 to arm the quadcopter
# Then set Mode 5 to start flying along the path

# For hand-gesture tracking
roslaunch openni_launch openni.launch
rosrun iris_sim detectRGBD.py # Launch hand detection script
# Put hand in the green rectangles and press 'z' to apply histogram mask
# When the detection succeeds, press 'a' to start tracking fingertip
rosrun iris_sim setMode.py
# First set Mode 1 to arm the quadcopter
# Then set Mode 6 to follow the fingertip trajectory
```
## 10. Relevant APIs in DJI MSDKv5 <a name="relevant-apis-in-dji-msdkv5"></a>

### Relevant API Integration

### Overview

DJI Mobile SDK v5 (MSDKv5) provides APIs for controlling DJI drones. For dynamically adding and editing waypoints, you would typically use the MissionControl feature of the SDK.

Here are some relevant APIs:

**WaypointMissionOperator:** This class provides methods for creating, managing, and executing waypoint missions.

**WaypointMissionBuilder:** Allows building and configuring waypoint missions programmatically.

**WaypointMission:** Represents a waypoint mission with parameters such as waypoints, speed, and actions at waypoints.

```java
// Create a WaypointMissionBuilder
WaypointMission.Builder builder = new WaypointMission.Builder();

// Add waypoints dynamically
builder.addWaypoint(new Waypoint(lat1, lon1, altitude));
builder.addWaypoint(new Waypoint(lat2, lon2, altitude));

// Set other mission parameters
builder.setFlightPathMode(WaypointMissionFlightPathMode.NORMAL);

// Build the mission
WaypointMission mission = builder.build();

// Execute the mission using WaypointMissionOperator
WaypointMissionOperator operator = DJISDKManager.getInstance().getMissionControl().getWaypointMissionOperator();
operator.uploadMission(mission, new CommonCallbacks.CompletionCallback() {
    @Override
    public void onResult(DJIError error) {
        if (error == null) {
            // Mission uploaded successfully, start the mission
            operator.startMission(new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError error) {
                    if (error != null) {
                        // Handle start mission error
                    }
                }
            });
        } else {
            // Handle upload mission error
        }
    }
});
```
The provided Java code snippet showcases the creation and execution of dynamic waypoint missions using DJI's Mobile SDK v5 on an Android platform. Initially, a `WaypointMission.Builder` object is instantiated to construct the mission. Waypoints are dynamically added to the mission using the `addWaypoint()` method, with each waypoint defined by latitude, longitude, and altitude coordinates. Additional mission parameters, such as flight path mode, are set using methods like `setFlightPathMode()`. Once the mission is fully configured, it is built into a `WaypointMission` object. Subsequently, the mission is uploaded to the drone using the `uploadMission()` method of the `WaypointMissionOperator`. Upon successful upload, the mission is started using the `startMission()` method. Error handling is implemented within callback functions to manage potential issues during mission upload and execution, ensuring smooth operation and providing appropriate feedback to the user. Overall, this code snippet offers a concise yet comprehensive demonstration of dynamically managing waypoint missions for DJI drones within an Android environment.
