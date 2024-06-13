# ClearSpot AI - ROS2 Crash Course

## Table of Contents

1. [Introduction to ROS 2](#introduction)
    - [Overview of ROS 2](#rrt)
    - [Differences between ROS 1 and ROS 2](#pilot)
    - [Key Features and Benefits](#tracking)
3. [Background](#background)
4. [Installation](#nodes)
    - [ubuntu](#rrt)
5. [Setting Up a Development Environment](#important-tip)
    - [Recommended IDEs and tools](#rrt)
    - [Setting up version control](#pilot)
    - [Configuring build tools](#tracking)
   
7. [Basic Concepts](#installation)
    - [Nodes](#prerequisites)
    - [Discovery](#installing-px4-firmware)
    - [Interfaces](#installing-mavros-package)
    - [Topics](#installing-openni)
    - [Services](#prerequisites)
    - [Actions](#installing-px4-firmware)
    - [Parameters](#installing-mavros-package)
    - [Introspection with command line tools](#installing-openni)
    - [Launch](#installing-openni)
    - [Client libraries](#installing-openni)
8. [Intermediate Concepts](#running-the-simulation)
    - [The ROS_DOMAIN_ID](#prerequisites)
    - [Different ROS 2 middleware vendors](#installing-px4-firmware)
    - [Logging and logger configuration](#installing-mavros-package)
    - [Quality of Service settings](#installing-openni)
    - [Executors](#prerequisites)
    - [Topic statistics](#installing-px4-firmware)
    - [Overview and usage of RQt](#installing-mavros-package)
    - [Introspection with command line tools](#installing-openni)
    - [Composition](#installing-openni)
    - [Cross-compilation](#installing-openni)
    - [ROS 2 Security](#installing-openni)
    - [Tf2](#installing-openni)
10. [Stretch Goal](#stretch-goal)
11. [KMZ Structure and Waypoint Editing Script](#kmz-structure-and-waypoint-editing-script)
12. [Simulating with Gazebo](#simulating-with-gazebo)
13. [Relevant APIs in DJI MSDKv5](#relevant-apis-in-dji-msdkv5)


## 1. Introduction to ROS 2 <a name="introduction"></a>

### Overview of ROS 2
ROS 2 (Robot Operating System 2) is an open-source framework for building robot applications. It provides the tools, libraries, and conventions needed to create complex and robust robot behaviors. ROS 2 is a significant improvement over its predecessor, ROS 1, with a focus on addressing the limitations and challenges faced by developers in ROS 1.

### Differences between ROS 1 and ROS 2

Next, Let's see the differences between ROS 1 and ROS 2.

#### Differences Between ROS 1 and ROS 2

| Feature                  | ROS 1                                     | ROS 2                                        |
|--------------------------|-------------------------------------------|----------------------------------------------|
| **Middleware**           | Custom-built middleware                   | DDS (Data Distribution Service)              |
| **Real-Time Capabilities**| Limited support                           | Designed for real-time applications          |
| **Security**             | Basic security measures                   | Advanced security (encryption, access control)|
| **Platform Support**     | Primarily Ubuntu                          | Cross-platform (Ubuntu, Windows, macOS)      |
| **Communication**        | Custom protocols                          | Standardized DDS protocols with QoS settings |
| **Modularity**           | Less modular                              | Highly modular                               |
| **Multi-Robot Support**  | Limited                                   | Improved support for multi-robot systems     |
| **Tooling**              | Basic debugging and visualization tools   | Advanced tools for debugging, simulation, and visualization (e.g., Gazebo, RViz) |
| **Package Management**   | `catkin` build system                     | `colcon` build tool                          |
| **Community and Ecosystem**| Extensive community with many packages   | Growing community with modernized packages   |
| **Launch System**        | XML-based launch files                    | Python-based launch files                    |
| **Node Lifecycle Management**| Limited support                      | Full node lifecycle management               |
| **Parameter Handling**   | Basic parameter server                    | Enhanced parameter handling                  |
| **Quality of Service (QoS)**| Not available                          | Customizable QoS policies                    |
| **Networking**           | Custom networking setup                   | Native support for multi-host networking     |
| **Dependency Management**| Manual handling                           | Improved dependency management               |

### Key Features and Benefits
1. **Middleware Abstraction**:
   - **DDS (Data Distribution Service)**: ROS 2 uses DDS, a standard for real-time data communication, which enhances performance, reliability, and scalability.
   - **Quality of Service (QoS)**: Customizable QoS policies allow for fine-tuned control over communication, essential for real-time and critical applications.

2. **Real-Time Support**:
   - ROS 2 is designed with real-time systems in mind, enabling developers to build applications that require deterministic behavior and timely execution.

3. **Security**:
   - Enhanced security features include secure communication, authentication, and encryption, ensuring that robotic systems can be protected against various threats.

4. **Cross-Platform Compatibility**:
   - ROS 2 supports multiple operating systems, including Ubuntu, Windows, and macOS, making it more versatile and accessible to a wider range of developers and use cases.

5. **Modular and Flexible Architecture**:
   - ROS 2 offers a modular architecture that allows developers to use only the components they need, making it easier to integrate with other systems and technologies.

6. **Improved Tooling**:
   - Advanced tools for debugging, simulation (such as Gazebo), and visualization (such as RViz) enhance the development process, making it more efficient and effective.

7. **Better Support for Multi-Robot Systems**:
   - ROS 2 includes features that simplify the development and deployment of multi-robot systems, including improved communication and coordination mechanisms.

## 2. Background <a name="background"></a>
In order to understand the significance and evolution of ROS 2, it's helpful to have some context about the origins and development of the Robot Operating System (ROS) ecosystem.
### Evolution of ROS 1

ROS 1 was introduced in 2007 by researchers at Willow Garage as a flexible framework for robotic software development. It quickly gained popularity due to its open-source nature, modular architecture, and rich set of libraries and tools. ROS 1 revolutionized the way robotic systems were developed by providing a standardized platform for sharing code, building complex behaviors, and integrating sensors and actuators.

Over the years, ROS 1 grew into a vibrant ecosystem with a large community of developers contributing packages, libraries, and tools for various robotic applications. However, as robotic systems became more complex and diverse, several limitations of ROS 1 became apparent, prompting the need for a more advanced and scalable framework.

### Development of ROS 2

The development of ROS 2 began in 2013 as a collaboration between various organizations and individuals within the ROS community. The goal was to address the shortcomings of ROS 1 and to create a framework that could meet the requirements of modern robotic applications.

ROS 2 was designed with a focus on reliability, scalability, and real-time performance, making it suitable for a wide range of applications, from research and academia to industrial automation and commercial robotics. By adopting new technologies and standards, such as the Data Distribution Service (DDS) for communication and improved security features, ROS 2 offers significant improvements over its predecessor.

### Adoption and Future Outlook

Since its initial release, ROS 2 has gained momentum within the robotics community, with an increasing number of projects and applications migrating from ROS 1 to ROS 2. The ROS 2 ecosystem continues to grow, with ongoing development efforts focused on enhancing performance, expanding platform support, and improving developer experience.

As the field of robotics continues to evolve, ROS 2 is expected to play a central role in shaping the future of robotic software development. Its modular architecture, cross-platform compatibility, and advanced features make it a powerful tool for building the next generation of robotic systems.
## 3. Installation <a name="nodes"></a>
Jazzy Jalisco is primarily supported on the following platforms:

### Tier 1 platforms:

- Ubuntu 24.04 (Noble): amd64 and arm64

- Windows 10 (Visual Studio 2019): amd64

### Tier 2 platforms:

- RHEL 9: amd64

### Tier 3 platforms:

- macOS: amd64

- Debian Bookworm: amd64

## Ubuntu Installation <a name="rrt"></a>

### Set Locale 

Make sure you have a locale which supports `UTF-8`. If you are in a minimal environment (such as a docker container), the locale may be something minimal like `POSIX`. We test with the following settings. However, it should be fine if you’re using a different UTF-8 supported locale.
```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
### Enable Required Repositories
You will need to add the ROS 2 apt repository to your system.

First ensure that the [Ubuntu Universe repository](https://help.ubuntu.com/community/Repositories/Ubuntu) is enabled.

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```
Now add the ROS 2 GPG key with apt.

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
Then add the repository to your source list.

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install development tools (optional)
If you are going to build ROS packages or otherwise do development, you can also install the development tools:

```bash
sudo apt update && sudo apt install ros-dev-tools
```

### Install ROS 2

Update your apt repository caches after setting up the repositories.

```bash
sudo apt update
```
ROS 2 packages are built on frequently updated Ubuntu systems. It is always recommended that you ensure your system is up to date before installing new packages.

```bash
sudo apt upgrade
```
Desktop Install (Recommended): ROS, RViz, demos, tutorials.

```bash
sudo apt install ros-jazzy-desktop
```

### Setup environment

Set up your environment by sourcing the following file.
```bash
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/jazzy/setup.bash
```
### Try Some Examples
If you installed `ros-jazzy-desktop` above you can try some examples.

In one terminal, source the setup file and then run a `C++ talker`:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
```
In another terminal source the setup file and then run a `Python listener`:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py listener
```

You should see the `talker` saying that it’s `Publishing` messages and the `listener` saying `I heard` those messages. This verifies both the C++ and Python APIs are working properly. Hooray!

## 4. Setting Up a Development Environment <a name="important-tip"></a>
Setting up a development environment for ROS 2 involves configuring your system to build and develop ROS packages. This section provides instructions for installing the necessary tools and configuring your environment.

### Recommended IDEs and Tools
While ROS 2 development can be done using any text editor and terminal, using an Integrated Development Environment (IDE) can greatly enhance productivity. Here are some recommended IDEs and tools for ROS 2 development:

- **Visual Studio Code**: A popular and lightweight IDE with built-in support for CMake, Python, and many other programming languages. Install the "ROS" extension for ROS-specific features.
- **CLion**: A powerful IDE developed by JetBrains, with advanced C++ support and seamless integration with CMake. Install the "ROS Support" plugin for ROS development.
- **Eclipse**: A versatile IDE with support for C/C++ development. Install the "ROS Eclipse" plugin for ROS development.

Additionally, consider installing the following tools to streamline your development workflow:

- **Git**: Version control system for managing your ROS packages and collaborating with others.
- **CMake**: Build system used by ROS to compile packages.
- **Python**: Programming language commonly used for ROS nodes and scripts.
- **Gazebo**: Robot simulation environment for testing and debugging robotic applications.

### Setting Up Version Control
If you haven't already, install Git on your system to manage your ROS packages using version control. You can install Git using your system's package manager:

```bash
sudo apt update && sudo apt install git
```
Once Git is installed, configure your username and email address:
```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```
### Configuring Build Tools 
ROS 2 packages use CMake as the build system. Make sure you have CMake installed on your system:
```bash
sudo apt update && sudo apt install cmake
```
Additionally, ROS 2 provides development tools for building and managing packages. You can install these tools using the following command:

```bash
sudo apt update && sudo apt install ros-dev-tools
```
These tools include essential utilities for ROS 2 development, such as `rosdep` for managing package dependencies, `colcon` for building packages, and `ament_lint` for code quality checks.

With your development environment set up, you're ready to start building and developing ROS 2 packages. Use your preferred IDE or text editor to create and modify ROS nodes, and leverage the ROS tools for building, testing, and debugging your applications.
## 5. Basic Concepts <a name="installation"></a>

### Nodes <a name="prerequisites"></a>

A node is a participant in the ROS 2 graph, which uses a client library to communicate with other nodes. Nodes can communicate with other nodes within the same process, in a different process, or on a different machine. Nodes are typically the unit of computation in a ROS graph; each node should do one logical thing.

Nodes can publish to named topics to deliver data to other nodes, or subscribe to named topics to get data from other nodes. They can also act as a service client to have another node perform a computation on their behalf, or as a service server to provide functionality to other nodes. For long-running computations, a node can act as an action client to have another node perform it on their behalf, or as an action server to provide functionality to other nodes. Nodes can provide configurable parameters to change behavior during run-time.

Nodes are often a complex combination of publishers, subscribers, service servers, service clients, action servers, and action clients, all at the same time.

Connections between nodes are established through a distributed discovery process.



### Discovery <a name="installing-px4-firmware"></a>

Discovery of nodes happens automatically through the underlying middleware of ROS 2. It can be summarized as follows:

1. When a node is started, it advertises its presence to other nodes on the network with the same ROS domain (set with the ROS_DOMAIN_ID environment variable). Nodes respond to this advertisement with information about themselves so that the appropriate connections can be made and the nodes can communicate.

2. Nodes periodically advertise their presence so that connections can be made with new-found entities, even after the initial discovery period.

3. Nodes advertise to other nodes when they go offline.

Nodes will only establish connections with other nodes if they have compatible Quality of Service settings.

Take the talker-listener demo for example. Running the C++ talker node in one terminal will publish messages on a topic, and the Python listener node running in another terminal will subscribe to messages on the same topic.

You should see that these nodes discover each other automatically, and begin to exchange messages.

### Interfaces <a name="installing-mavros-package"></a>
## Table of Contents

- [Background](#rrt)
- [Messages](#pilot)
    - [Fields](#rrt)
        - [Field types](#rrt)
        - [Field names](#rrt)
        - [Field default value](#rrt)
    - [Constants](#rrt)
- [Services](#pilot)
- [Actions](#tracking)

    

### Background
ROS applications typically communicate through interfaces of one of three types: topics, services, or actions. ROS 2 uses a simplified description language, the interface definition language (IDL), to describe these interfaces. This description makes it easy for ROS tools to automatically generate source code for the interface type in several target languages.

In this document we will describe the supported types:

msg: `.msg` files are simple text files that describe the fields of a ROS message. They are used to generate source code for messages in different languages.

srv: `.srv` files describe a service. They are composed of two parts: a request and a response. The request and response are message declarations.

action: `.action` files describe actions. They are composed of three parts: a goal, a result, and feedback. Each part is a message declaration itself.

### Messages
Messages are a way for a ROS 2 node to send data on the network to other ROS nodes, with no response expected. For instance, if a ROS 2 node reads temperature data from a sensor, it can then publish that data on the ROS 2 network using a `Temperature` message. Other nodes on the ROS 2 network can subscribe to that data and receive the `Temperature` message.

Messages are described and defined in `.msg` files in the `msg/` directory of a ROS package. `.msg` files are composed of two parts: fields and constants.

### Fields
Each field consists of a type and a name, separated by a space, i.e:
```bash
fieldtype1 fieldname1
fieldtype2 fieldname2
fieldtype3 fieldname3
```
For example:
```bash
int32 my_int
string my_string
```
### Field Types
Field types can be:

- a built-in-type

- names of Message descriptions defined on their own, such as “geometry_msgs/PoseStamped”

Built-in-types currently supported:

| Type Name  | C++                 | Python             | DDS Type           |
|------------|---------------------|--------------------|--------------------|
| bool       | bool                | bool               | builtins.bool      |
| byte       | uint8_t             | builtins.bytes*    | octet              |
| char       | char                | builtins.int*      | char               |
| float32    | float               | builtins.float*    | float              |
| float64    | double              | builtins.float*    | double             |
| int8       | int8_t              | builtins.int*      | octet              |
| uint8      | uint8_t             | builtins.int*      | octet              |
| int16      | int16_t             | builtins.int*      | short              |
| uint16     | uint16_t            | builtins.int*      | unsigned short     |
| int32      | int32_t             | builtins.int*      | long               |
| uint32     | uint32_t            | builtins.int*      | unsigned long      |
| int64      | int64_t             | builtins.int*      | long long          |
| uint64     | uint64_t            | builtins.int*      | unsigned long long |
| string     | std::string         | builtins.str       | string             |
| wstring    | std::u16string      | builtins.str       | wstring            |

Every built-in-type can be used to define arrays:

| Type Name            | C++                   | Python             | DDS Type           |
|----------------------|-----------------------|--------------------|--------------------|
| static array         | std::array<T, N>                  |builtins.list*                 | T[N]               |
| unbounded dynamic array | std::vector<T>     | builtins.list      | sequence                  |
| bounded dynamic array | custom_class<T, N>  | builtins.list*     | sequence<T, N>     |
| bounded string       | std::string           | builtins.str*      | string             |

All types that are more permissive than their ROS definition enforce the ROS constraints in range and length by software.

Example of message definition using arrays and bounded types:

```
int32[] unbounded_integer_array
int32[5] five_integers_array
int32[<=5] up_to_five_integers_array

string string_of_unbounded_size
string<=10 up_to_ten_characters_string

string[<=5] up_to_five_unbounded_strings
string<=10[] unbounded_array_of_strings_up_to_ten_characters_each
string<=10[<=5] up_to_five_strings_up_to_ten_characters_each
```

### Field Names
Field names must be lowercase alphanumeric characters with underscores for separating words. They must start with an alphabetic character, and they must not end with an underscore or have two consecutive underscores.

### Field Default Value
Default values can be set to any field in the message type. Currently default values are not supported for string arrays and complex types (i.e. types not present in the built-in-types table above; that applies to all nested messages).

Defining a default value is done by adding a third element to the field definition line, i.e:
```
fieldtype fieldname fielddefaultvalue
```
For example:
```
uint8 x 42
int16 y -2000
string full_name "John Doe"
int32[] samples [-200, -100, 0, 100, 200]
```
**Note :**
- string values must be defined in single ' or double " quotes

- currently string values are not escaped

### Constants
Each constant definition is like a field description with a default value, except that this value can never be changed programatically. This value assignment is indicated by use of an equal ‘=’ sign, e.g.
```
constanttype CONSTANTNAME=constantvalue
```
For Example :
```
int32 X=123
int32 Y=-123
string FOO="foo"
string EXAMPLE='bar'
```
**Note :**
- Constants names have to be UPPERCASE
### Services

Services are a request/response communication, where the client (requester) is waiting for the server (responder) to make a short computation and return a result.

Services are described and defined in `.srv` files in the `srv/` directory of a ROS package.

A service description file consists of a request and a response msg type, separated by `---`. Any two `.msg` files concatenated with a `---` are a legal service description.

Here is a very simple example of a service that takes in a string and returns a string:
```
string str
---
string str
```
We can of course get much more complicated (if you want to refer to a message from the same package you must not mention the package name):
```
# request constants
int8 FOO=1
int8 BAR=2
# request fields
int8 foobar
another_pkg/AnotherMessage msg
---
# response constants
uint32 SECRET=123456
# response fields
another_pkg/YetAnotherMessage val
CustomMessageDefinedInThisPackage value
uint32 an_integer
```
You cannot embed another service inside of a service.
### Actions
Actions are a long-running request/response communication, where the action client (requester) is waiting for the action server (the responder) to take some action and return a result. In contrast to services, actions can be long-running (many seconds or minutes), provide feedback while they are happening, and can be interrupted.

Action definitions have the following form:
```
<request_type> <request_fieldname>
---
<response_type> <response_fieldname>
---
<feedback_type> <feedback_fieldname>
```
Like services, the request fields are before and the response fields are after the first triple-dash `(---)`, respectively. There is also a third set of fields after the second triple-dash, which is the fields to be sent when sending feedback.

There can be arbitrary numbers of request fields (including zero), arbitrary numbers of response fields (including zero), and arbitrary numbers of feedback fields (including zero).

The `<request_type>`, `<response_type>`, and `<feedback_type>` follow all of the same rules as the `<type>` for a message. The `<request_fieldname>`, `<response_fieldname>`, and `<feedback_fieldname>` follow all of the same rules as the `<fieldname>` for a message.

For instance, the `Fibonacci` action definition contains the following:
```
int32 order
---
int32[] sequence
---
int32[] sequence
```
This is an action definition where the action client is sending a single `int32` field representing the number of Fibonacci steps to take, and expecting the action server to produce an array of `int32` containing the complete steps. Along the way, the action server may also provide an intermediate array of `int32` containing the steps accomplished up until a certain point.

### Topics
**Table of Contents**
- [Publish/Subscribe](#rrt)
- [Anonymous](#pilot)
- [Strongly-typed](#tracking)

Topics are one of the three primary styles of interfaces provided by ROS 2. Topics should be used for continuous data streams, like sensor data, robot state, etc.

As stated earlier, ROS 2 is a strongly-typed, anonymous publish/subscribe system. Let’s break down that sentence and explain it a bit more.
### Publish/Subscribe
A publish/subscribe system is one in which there are producers of data (publishers) and consumers of data (subscribers). The publishers and subscribers know how to contact each other through the concept of a “topic”, which is a common name so that the entites can find each other. For instance, when you create a publisher, you must also give it a string that is the name of the topic; the same goes for the subscriber. Any publishers and subscribers that are on the same topic name can directly communicate with each other. There may be zero or more publishers and zero or more subscribers on any particular topic. When data is published to the topic by any of the publishers, all subscribers in the system will receive the data. This system is also known as a “bus”, since it somewhat resembles a device bus from electrical engineering. This concept of a bus is part of what makes ROS 2 a powerful and flexible system. Publishers and subscribers can come and go as needed, meaning that debugging and introspection are natural extensions to the system. For instance, if you want to record data, you can use the `ros2 bag record` command. Under the hood, `ros2 bag record` creates a new subscriber to whatever topic you tell it, without interrupting the flow of data to the other parts of the system.

### Anonymous
Another fact mentioned in the introduction is that ROS 2 is “anonymous”. This means that when a subscriber gets a piece of data, it doesn’t generally know or care which publisher originally sent it (though it can find out if it wants). The benefit to this architecture is that publishers and subscribers can be swapped out at will without affecting the rest of the system.
### Strongly Typed
Finally, the introduction also mentioned that the publish/subscribe system is “strongly-typed”. That has two meanings in this context:

1. The types of each field in a ROS message are typed, and that type is enforced at various levels. For instance, if the ROS message contains:
    ```
    uint32 field1
    string field2
    ```
    Then the code will ensure that `field1` is always an unsigned integer and that `field2` is always a string.
2.  The semantics of each field are well-defined. There is no automated mechanism to ensure this, but all of the core ROS types have strong semantics associated with them. For instance, the IMU message contains a 3-dimensional vector for the measured angular velocity, and each of the dimensions is specified to be in radians/second. Other interpretations should not be placed into the message.

### Services
## Table of Contents

- [Service Server](#service_server)
- [Service Client](#service_client)


In ROS 2, a service refers to a remote procedure call. In other words, a node can make a remote procedure call to another node which will do a computation and return a result.

This structure is reflected in how a service message definition looks:
```
uint32 request
---
uint32 response
```
In ROS 2, services are expected to return quickly, as the client is generally waiting on the result. Services should never be used for longer running processes, in particular processes that might need to be preempted for exceptional situations. If you have a service that will be doing a long-running computation, consider using an action instead.

Services are identified by a service name, which looks much like a topic name (but is in a different namespace).

A service consists of two parts: the service server and the service client.

### Service Server <a name="service_server"></a>
A service server is the entity that will accept a remote procedure request, and perform some computation on it. For instance, suppose the ROS 2 message contains the following:
```
uint32 a
uint32 b
---
uint32 sum
```
The service server would be the entity that receives this message, adds `a` and `b` together, and returns the `sum`.

**Note**
There should only ever be one service server per service name. It is undefined which service server will receive client requests in the case of multiple service servers on the same service name.

### Service Client <a name="service_client"></a>

A service client is an entity that will request a remote service server to perform a computation on its behalf. Following from the example above, the service client is the entity that creates the initial message containing `a` and `b`, and waits for the service server to compute the sum and return the result.

Unlike the service server, there can be arbitrary numbers of service clients using the same service name.

### Actions
## Table of Contents

- [Action Server](#action_server)
- [Action Client](#action_client)

In ROS 2, an action refers to a long-running remote procedure call with feedback and the ability to cancel or preempt the goal. For instance, the high-level state machine running a robot may call an action to tell the navigation subsystem to travel to a waypoint, which may take several seconds (or minutes) to do. Along the way, the navigation subsystem can provide feedback on how far along it is, and the high-level state machine has the option to cancel or preempt the travel to that waypoint.

This structure is reflected in how an action message definition looks:
```
int32 request
---
int32 response
---
int32 feedback
```
In ROS 2, actions are expected to be long running procedures, as there is overhead in setting up and monitoring the connection. If you need a short running remote procedure call, consider using a service instead.

Actions are identified by an action name, which looks much like a topic name (but is in a different namespace).

An action consists of two parts: the action server and the action client.

### Action Server <a name="action_server"></a>
The action server is the entity that will accept the remote procedure request and perform some procedure on it. It is also responsible for sending out feedback as the action progresses and should react to cancellation/preemption requests. For instance, consider an action to calculate the Fibonacci sequence with the following interface:

```
int32 order
---
int32[] sequence
---
int32[] sequence
```
The action server is the entity that receives this message, starts calculating the sequence up to `order` (providing feedback along the way), and finally returns a full result in `sequence`.

**Note**
There should only ever be one action server per action name. It is undefined which action server will receive client requests in the case of multiple action servers on the same action name.

### Action Client <a name="action_client"></a>

An action client is an entity that will request a remote action server to perform a procedure on its behalf. Following the example above, the action client is the entity that creates the initial message containing the `order`, and waits for the action server to compute the sequence and return it (with feedback along the way).

Unlike the action server, there can be arbitrary numbers of action clients using the same action name.
   
### Parameters
## Table of Contents

- [Overview](#paramover)
- [Parameters Background](#parambg)
  - [Declaring parameters](#decparam)
  - [Parameter types](#paramtypes)
  - [Parameter callbacks](#paramcb)
- [Interacting with parameters](#
- Setting initial parameter values when running a node
- Setting initial parameter values when launching nodes
- Manipulating parameter values at runtime
- Migrating from ROS 1

### Overview
Parameters in ROS 2 are associated with individual nodes. Parameters are used to configure nodes at startup (and during runtime), without changing the code. The lifetime of a parameter is tied to the lifetime of the node (though the node could implement some sort of persistence to reload values after restart).

Parameters are addressed by node name, node namespace, parameter name, and parameter namespace. Providing a parameter namespace is optional.

Each parameter consists of a key, a value, and a descriptor. The key is a string and the value is one of the following types: `bool`, `int64`, `float64`, `string`, `byte[]`, `bool[]`, `int64[]`, `float64[]` or `string[]`. By default all descriptors are empty, but can contain parameter descriptions, value ranges, type information, and additional constraints.

For a hands-on tutorial with ROS parameters see [Understanding parameters](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html).

### Parameters Backgroud
### Declaring Parameters
By default, a node needs to declare all of the parameters that it will accept during its lifetime. This is so that the type and name of the parameters are well-defined at node startup time, which reduces the chances of misconfiguration later on. 

For some types of nodes, not all of the parameters will be known ahead of time. In these cases, the node can be instantiated with `allow_undeclared_parameters` set to `true`, which will allow parameters to be get and set on the node even if they haven’t been declared.
### Parameter Types
Each parameter on a ROS 2 node has one of the pre-defined parameter types as mentioned in the Overview. By default, attempts to change the type of a declared parameter at runtime will fail. This prevents common mistakes, such as putting a boolean value into an integer parameter.

If a parameter needs to be multiple different types, and the code using the parameter can handle it, this default behavior can be changed. When the parameter is declared, it should be declared using a `ParameterDescriptor` with the `dynamic_typing` member variable set to `true`.
### Parameter Callbacks
A ROS 2 node can register three different types of callbacks to be informed when changes are happening to parameters. All three of the callbacks are optional.

The first is known as a “pre set parameter” callback, and can be set by calling `add_pre_set_parameters_callback` from the node API. This callback is passed a list of the `Parameter` objects that are being changed, and returns nothing. When it is called, it can modify the `Parameter` list to change, add, or remove entries. As an example, if `parameter2` should change anytime that `parameter1` changes, that can be implemented with this callback.

The second is known as a “set parameter” callback, and can be set by calling `add_on_set_parameters_callback` from the node API. The callback is passed a list of immutable `Parameter` objects, and returns an `rcl_interfaces/msg/SetParametersResult`. The main purpose of this callback is to give the user the ability to inspect the upcoming change to the parameter and explicitly reject the change.

**Note**

It is important that “set parameter” callbacks have no side-effects. Since multiple “set parameter” callbacks can be chained, there is no way for an individual callback to know if a later callback will reject the update. If the individual callback were to make changes to the class it is in, for instance, it may get out-of-sync with the actual parameter. To get a callback after a parameter has been successfully changed, see the next type of callback below.

The third type of callback is known as an “post set parameter” callback, and can be set by calling `add_post_set_parameters_callback` from the node API. The callback is passed a list of immutable `Parameter` objects, and returns nothing. The main purpose of this callback is to give the user the ability to react to changes from parameters that have successfully been accepted.

### Interacting with Parameters

ROS 2 nodes can perform parameter operations through node APIs as described in Using parameters in a class (C++) or Using parameters in a class (Python). External processes can perform parameter operations via parameter services that are created by default when a node is instantiated. The services that are created by default are:

- `/node_name/describe_parameters`: Uses a service type of `rcl_interfaces/srv/DescribeParameters`. Given a list of parameter names, returns a list of descriptors associated with the parameters.

- `/node_name/get_parameter_types`: Uses a service type of `rcl_interfaces/srv/GetParameterTypes`. Given a list of parameter names, returns a list of parameter types associated with the parameters.

- `/node_name/get_parameters`: Uses a service type of `rcl_interfaces/srv/GetParameters`. Given a list of parameter names, returns a list of parameter values associated with the parameters.

- `/node_name/list_parameters`: Uses a service type of `rcl_interfaces/srv/ListParameters`. Given an optional list of parameter prefixes, returns a list of the available parameters with that prefix. If the prefixes are empty, returns all parameters.

- `/node_name/set_parameters`: Uses a service type of `rcl_interfaces/srv/SetParameters`. Given a list of parameter names and values, attempts to set the parameters on the node. Returns a list of results from trying to set each parameter; some of them may have succeeded and some may have failed.

- `/node_name/set_parameters_atomically`: Uses a service type of `rcl_interfaces/srv/SetParametersAtomically`. Given a list of parameter names and values, attempts to set the parameters on the node. Returns a single result from trying to set all parameters, so if one failed, all of them failed.

### Setting initial parameter values when running a node
Initial parameter values can be set when running the node either through individual command-line arguments, or through YAML files. See Setting parameters directly from the command line for examples on how to set initial parameter values.

### Setting initial parameter values when launching nodes
Initial parameter values can also be set when running the node through the ROS 2 launch facility. See this document for information on how to specify parameters via launch.

### Manipulating parameter values at runtime
The `ros2 param` command is the general way to interact with parameters for nodes that are already running. `ros2 param` uses the parameter service API as described above to perform the various operations. See this how-to guide for details on how to use `ros2 param`.

### Migrating from ROS 1
The [Launch file migration guide](https://docs.ros.org/en/jazzy/How-To-Guides/Migrating-from-ROS1/Migrating-Launch-Files.html) explains how to migrate `param` and `rosparam` launch tags from ROS 1 to ROS 2.

The [YAML parameter file migration guide](https://docs.ros.org/en/jazzy/How-To-Guides/Migrating-from-ROS1/Migrating-Parameters.html) explains how to migrate parameter files from ROS 1 to ROS 2.

In ROS 1, the `roscore` acted like a global parameter blackboard where all nodes could get and set parameters. Since there is no central `roscore` in ROS 2, that functionality no longer exists. The recommended approach in ROS 2 is to use per-node parameters that are closely tied to the nodes that use them. If a global blackboard is still needed, it is possible to create a dedicated node for this purpose. ROS 2 ships with one in the `ros-jazzy-demo-nodes-cpp` package called `parameter_blackboard`; it can be run with:
```
ros2 run demo_nodes_cpp parameter_blackboard
```
The code for the `parameter_blackboard` is here.

### Introspection with command line tools
### Table of Contents
- Usage
- Example
- Behind the Scenes
- Implementation

ROS 2 includes a suite of command-line tools for introspecting a ROS 2 system.

### Usage
The main entry point for the tools is the command `ros2`, which itself has various sub-commands for introspecting and working with nodes, topics, services, and more.

To see all available sub-commands run:
```
ros2 --help
```
Examples of sub-commands that are available include:


- `action`: Introspect/interact with ROS actions
- `bag`: Record/play a rosbag
- `component`: Manage component containers
- `daemon`: Introspect/configure the ROS 2 daemon
- `doctor`: Check ROS setup for potential issues
- `interface`: Show information about ROS interfaces
- `launch`: Run/introspect a launch file
- `lifecycle`: Introspect/manage nodes with managed lifecycles
- `multicast`: Multicast debugging commands
- `node`: Introspect ROS nodes
- `param`: Introspect/configure parameters on a node
- `pkg`: Introspect ROS packages
- `run`: Run ROS nodes
- `security`: Configure security settings
- `service`: Introspect/call ROS services
- `test`: Run a ROS launch test
- `topic`: Introspect/publish ROS topics
- `trace`: Tracing tools to get information on ROS nodes execution (only available on Linux)
- `wtf`: An alias for doctor

### Example

To produce the typical talker-listener example using command-line tools, the topic sub-command can be used to publish and echo messages on a `topic`.

Publish messages in one terminal with:
```
$ ros2 topic pub /chatter std_msgs/msg/String "data: Hello world"
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='Hello world')

publishing #2: std_msgs.msg.String(data='Hello world')
```
Echo messages received in another terminal with:
```
$ ros2 topic echo /chatter
data: Hello world

data: Hello world
```
### Behind the scenes

ROS 2 uses a distributed discovery process for nodes to connect to each other. As this process purposefully does not use a centralized discovery mechanism, it can take time for ROS nodes to discover all other participants in the ROS graph. Because of this, there is a long-running daemon in the background that stores information about the ROS graph to provide faster responses to queries, e.g. the list of node names.

The daemon is automatically started when the relevant command-line tools are used for the first time. You can run `ros2 daemon --help` for more options for interacting with the daemon.

### Implementation

The source code for the `ros2 command` is available at https://github.com/ros2/ros2cli.

The `ros2` tool has been implemented as a framework that can be extended via plugins. For example, the [sros2](https://github.com/ros2/sros2) package provides a `security` sub-command that is automatically detected by the `ros2` tool if the `sros2` package is installed.

### Launch
A ROS 2 system typically consists of many nodes running across many different processes (and even different machines). While it is possible to run each of these nodes separately, it gets cumbersome quite quickly.

The launch system in ROS 2 is meant to automate the running of many nodes with a single command. It helps the user describe the configuration of their system and then executes it as described. The configuration of the system includes what programs to run, where to run them, what arguments to pass them, and ROS-specific conventions which make it easy to reuse components throughout the system by giving them each a different configuration. It is also responsible for monitoring the state of the processes launched, and reporting and/or reacting to changes in the state of those processes.

All of the above is specified in a launch file, which can be written in Python, XML, or YAML. This launch file can then be run using the `ros2 launch` command, and all of the nodes specified will be run.

The [design document](https://design.ros2.org/articles/roslaunch.html) details the goal of the design of ROS 2’s launch system (not all functionality is currently available).
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
