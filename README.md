# ClearSpot AI - ROS2 Crash Course

## Table of Contents

1. [Introduction to ROS 2](#introros2)
    - [Overview of ROS 2](#introveriew)
    - [Differences between ROS 1 and ROS 2](#diffr1r2)
    - [Key Features and Benefits](#kfab)
2. [Note](#note)
3. [Background](#background)
4. [Installation](#instl)
    - [ubuntu](#ubu)
5. [Setting Up a Development Environment](#sde)
    - [Recommended IDEs and tools](#riat)
    - [Setting up version control](#suvc)
    - [Configuring build tools](#cbt)
   
6. [Basic Concepts](#bc)
    - [Nodes](#nodes)
    - [Discovery](#discovery)
    - [Interfaces](#interfaces)
    - [Topics](#topics)
    - [Services](#services)
    - [Actions](#actions)
    - [Parameters](#parameters)
    - [Introspection with command line tools](#intros)
    - [Launch](#launch)
    - [Client libraries](#clilib)
7. [Intermediate Concepts](#inter)
    - [The ROS_DOMAIN_ID](#rosdom)
    - [Different ROS 2 middleware vendors](#interdif)
    - [Logging and logger configuration](#interlog)
    
8. [Additional Reference Materials](#addref)
9. [Project Submission](#oa)
10. [Hands-On Exercises](#hoe)



## 1. Introduction to ROS 2 <a name="introros2"></a>

### Overview of ROS 2 <a name="introveriew"></a>
ROS 2 (Robot Operating System 2) is an open-source framework for building robot applications. It provides the tools, libraries, and conventions needed to create complex and robust robot behaviors. ROS 2 is a significant improvement over its predecessor, ROS 1, with a focus on addressing the limitations and challenges faced by developers in ROS 1.

### Differences between ROS 1 and ROS 2 <a name="diffr1r2"></a>

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

### Key Features and Benefits <a name="kfab"></a>
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

## 3. Background <a name="background"></a>
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
## 4. Installation <a name="instl"></a>
Jazzy Jalisco is primarily supported on the following platforms:

### Tier 1 platforms:

- Ubuntu 24.04 (Noble): amd64 and arm64

- Windows 10 (Visual Studio 2019): amd64

### Tier 2 platforms:

- RHEL 9: amd64

### Tier 3 platforms:

- macOS: amd64

- Debian Bookworm: amd64

## Ubuntu Installation <a name="ubu"></a>

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

## 5. Setting Up a Development Environment <a name="sde"></a>
Setting up a development environment for ROS 2 involves configuring your system to build and develop ROS packages. This section provides instructions for installing the necessary tools and configuring your environment.

### Recommended IDEs and Tools <a name="riat"></a>
While ROS 2 development can be done using any text editor and terminal, using an Integrated Development Environment (IDE) can greatly enhance productivity. Here are some recommended IDEs and tools for ROS 2 development:

- **Visual Studio Code**: A popular and lightweight IDE with built-in support for CMake, Python, and many other programming languages. Install the "ROS" extension for ROS-specific features.
- **CLion**: A powerful IDE developed by JetBrains, with advanced C++ support and seamless integration with CMake. Install the "ROS Support" plugin for ROS development.
- **Eclipse**: A versatile IDE with support for C/C++ development. Install the "ROS Eclipse" plugin for ROS development.

Additionally, consider installing the following tools to streamline your development workflow:

- **Git**: Version control system for managing your ROS packages and collaborating with others.
- **CMake**: Build system used by ROS to compile packages.
- **Python**: Programming language commonly used for ROS nodes and scripts.
- **Gazebo**: Robot simulation environment for testing and debugging robotic applications.

### Setting Up Version Control <a name="suvc"></a>
If you haven't already, install Git on your system to manage your ROS packages using version control. You can install Git using your system's package manager:

```bash
sudo apt update && sudo apt install git
```
Once Git is installed, configure your username and email address:
```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```
### Configuring Build Tools <a name="cbt"></a>
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
## 6. Basic Concepts <a name="bc"></a>

### Nodes <a name="nodes"></a>

A node is a participant in the ROS 2 graph, which uses a client library to communicate with other nodes. Nodes can communicate with other nodes within the same process, in a different process, or on a different machine. Nodes are typically the unit of computation in a ROS graph; each node should do one logical thing.

Nodes can publish to named topics to deliver data to other nodes, or subscribe to named topics to get data from other nodes. They can also act as a service client to have another node perform a computation on their behalf, or as a service server to provide functionality to other nodes. For long-running computations, a node can act as an action client to have another node perform it on their behalf, or as an action server to provide functionality to other nodes. Nodes can provide configurable parameters to change behavior during run-time.

Nodes are often a complex combination of publishers, subscribers, service servers, service clients, action servers, and action clients, all at the same time.

Connections between nodes are established through a distributed discovery process.



### Discovery <a name="discovery"></a>

Discovery of nodes happens automatically through the underlying middleware of ROS 2. It can be summarized as follows:

1. When a node is started, it advertises its presence to other nodes on the network with the same ROS domain (set with the ROS_DOMAIN_ID environment variable). Nodes respond to this advertisement with information about themselves so that the appropriate connections can be made and the nodes can communicate.

2. Nodes periodically advertise their presence so that connections can be made with new-found entities, even after the initial discovery period.

3. Nodes advertise to other nodes when they go offline.

Nodes will only establish connections with other nodes if they have compatible Quality of Service settings.

Take the talker-listener demo for example. Running the C++ talker node in one terminal will publish messages on a topic, and the Python listener node running in another terminal will subscribe to messages on the same topic.

You should see that these nodes discover each other automatically, and begin to exchange messages.

### Interfaces <a name="interfaces"></a>
## Table of Contents

- [Background](#ibg)
- [Messages](#imsg)
    - [Fields](#ifields)
        - [Field types](#ifieldtypes)
        - [Field names](#ifieldnames)
        - [Field default value](#ifielddefval)
    - [Constants](#iconst)
- [Services](#iservices)
- [Actions](#iactions)

    

### Background <a name="ibg"></a>
ROS applications typically communicate through interfaces of one of three types: topics, services, or actions. ROS 2 uses a simplified description language, the interface definition language (IDL), to describe these interfaces. This description makes it easy for ROS tools to automatically generate source code for the interface type in several target languages.

In this document we will describe the supported types:

msg: `.msg` files are simple text files that describe the fields of a ROS message. They are used to generate source code for messages in different languages.

srv: `.srv` files describe a service. They are composed of two parts: a request and a response. The request and response are message declarations.

action: `.action` files describe actions. They are composed of three parts: a goal, a result, and feedback. Each part is a message declaration itself.

### Messages <a name="imsg"></a>
Messages are a way for a ROS 2 node to send data on the network to other ROS nodes, with no response expected. For instance, if a ROS 2 node reads temperature data from a sensor, it can then publish that data on the ROS 2 network using a `Temperature` message. Other nodes on the ROS 2 network can subscribe to that data and receive the `Temperature` message.

Messages are described and defined in `.msg` files in the `msg/` directory of a ROS package. `.msg` files are composed of two parts: fields and constants.

### Fields <a name="ifields"></a>
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
### Field Types <a name="ifieldtypes"></a>
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

### Field Names <a name="ifieldnames"></a>
Field names must be lowercase alphanumeric characters with underscores for separating words. They must start with an alphabetic character, and they must not end with an underscore or have two consecutive underscores.

### Field Default Value <a name="ifielddefval"></a>
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

### Constants <a name="iconst"></a>
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
### Services <a name="iservices"></a>

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
### Actions <a name="iactions"></a>
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

### Topics <a name="topics"></a>
**Table of Contents**
- [Publish/Subscribe](#pub)
- [Anonymous](#anon)
- [Strongly-typed](#strong)

Topics are one of the three primary styles of interfaces provided by ROS 2. Topics should be used for continuous data streams, like sensor data, robot state, etc.

As stated earlier, ROS 2 is a strongly-typed, anonymous publish/subscribe system. Let’s break down that sentence and explain it a bit more.
### Publish/Subscribe <a name="pub"></a>
A publish/subscribe system is one in which there are producers of data (publishers) and consumers of data (subscribers). The publishers and subscribers know how to contact each other through the concept of a “topic”, which is a common name so that the entites can find each other. For instance, when you create a publisher, you must also give it a string that is the name of the topic; the same goes for the subscriber. Any publishers and subscribers that are on the same topic name can directly communicate with each other. There may be zero or more publishers and zero or more subscribers on any particular topic. When data is published to the topic by any of the publishers, all subscribers in the system will receive the data. This system is also known as a “bus”, since it somewhat resembles a device bus from electrical engineering. This concept of a bus is part of what makes ROS 2 a powerful and flexible system. Publishers and subscribers can come and go as needed, meaning that debugging and introspection are natural extensions to the system. For instance, if you want to record data, you can use the `ros2 bag record` command. Under the hood, `ros2 bag record` creates a new subscriber to whatever topic you tell it, without interrupting the flow of data to the other parts of the system.

### Anonymous <a name="anon"></a>
Another fact mentioned in the introduction is that ROS 2 is “anonymous”. This means that when a subscriber gets a piece of data, it doesn’t generally know or care which publisher originally sent it (though it can find out if it wants). The benefit to this architecture is that publishers and subscribers can be swapped out at will without affecting the rest of the system.
### Strongly Typed <a name="strong"></a>
Finally, the introduction also mentioned that the publish/subscribe system is “strongly-typed”. That has two meanings in this context:

1. The types of each field in a ROS message are typed, and that type is enforced at various levels. For instance, if the ROS message contains:
    ```
    uint32 field1
    string field2
    ```
    Then the code will ensure that `field1` is always an unsigned integer and that `field2` is always a string.
2.  The semantics of each field are well-defined. There is no automated mechanism to ensure this, but all of the core ROS types have strong semantics associated with them. For instance, the IMU message contains a 3-dimensional vector for the measured angular velocity, and each of the dimensions is specified to be in radians/second. Other interpretations should not be placed into the message.

### Services <a name="services"></a>
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

### Actions <a name="actions"></a>
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
   
### Parameters <a name="parameters"></a>
## Table of Contents

- [Overview](#paramover)
- [Parameters Background](#parambg)
  - [Declaring parameters](#decparam)
  - [Parameter types](#paramtypes)
  - [Parameter callbacks](#paramcb)
- [Interacting with parameters](#paramint)
- [Setting initial parameter values when running a node](#paramsetr)
- [Setting initial parameter values when launching nodes](#paramsetl)
- [Manipulating parameter values at runtime](#parammani)
- [Migrating from ROS 1](#parammig)

### Overview <a name="paramover"></a>
Parameters in ROS 2 are associated with individual nodes. Parameters are used to configure nodes at startup (and during runtime), without changing the code. The lifetime of a parameter is tied to the lifetime of the node (though the node could implement some sort of persistence to reload values after restart).

Parameters are addressed by node name, node namespace, parameter name, and parameter namespace. Providing a parameter namespace is optional.

Each parameter consists of a key, a value, and a descriptor. The key is a string and the value is one of the following types: `bool`, `int64`, `float64`, `string`, `byte[]`, `bool[]`, `int64[]`, `float64[]` or `string[]`. By default all descriptors are empty, but can contain parameter descriptions, value ranges, type information, and additional constraints.

For a hands-on tutorial with ROS parameters see [Understanding parameters](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html).

### Parameters Backgroud <a name="parambg"></a>
### Declaring Parameters <a name="decparam"></a>
By default, a node needs to declare all of the parameters that it will accept during its lifetime. This is so that the type and name of the parameters are well-defined at node startup time, which reduces the chances of misconfiguration later on. 

For some types of nodes, not all of the parameters will be known ahead of time. In these cases, the node can be instantiated with `allow_undeclared_parameters` set to `true`, which will allow parameters to be get and set on the node even if they haven’t been declared.
### Parameter Types <a name="paramtypes"></a>
Each parameter on a ROS 2 node has one of the pre-defined parameter types as mentioned in the Overview. By default, attempts to change the type of a declared parameter at runtime will fail. This prevents common mistakes, such as putting a boolean value into an integer parameter.

If a parameter needs to be multiple different types, and the code using the parameter can handle it, this default behavior can be changed. When the parameter is declared, it should be declared using a `ParameterDescriptor` with the `dynamic_typing` member variable set to `true`.
### Parameter Callbacks <a name="paramcb"></a>
A ROS 2 node can register three different types of callbacks to be informed when changes are happening to parameters. All three of the callbacks are optional.

The first is known as a “pre set parameter” callback, and can be set by calling `add_pre_set_parameters_callback` from the node API. This callback is passed a list of the `Parameter` objects that are being changed, and returns nothing. When it is called, it can modify the `Parameter` list to change, add, or remove entries. As an example, if `parameter2` should change anytime that `parameter1` changes, that can be implemented with this callback.

The second is known as a “set parameter” callback, and can be set by calling `add_on_set_parameters_callback` from the node API. The callback is passed a list of immutable `Parameter` objects, and returns an `rcl_interfaces/msg/SetParametersResult`. The main purpose of this callback is to give the user the ability to inspect the upcoming change to the parameter and explicitly reject the change.

**Note**

It is important that “set parameter” callbacks have no side-effects. Since multiple “set parameter” callbacks can be chained, there is no way for an individual callback to know if a later callback will reject the update. If the individual callback were to make changes to the class it is in, for instance, it may get out-of-sync with the actual parameter. To get a callback after a parameter has been successfully changed, see the next type of callback below.

The third type of callback is known as an “post set parameter” callback, and can be set by calling `add_post_set_parameters_callback` from the node API. The callback is passed a list of immutable `Parameter` objects, and returns nothing. The main purpose of this callback is to give the user the ability to react to changes from parameters that have successfully been accepted.

### Interacting with Parameters <a name="paramint"></a>

ROS 2 nodes can perform parameter operations through node APIs as described in Using parameters in a class (C++) or Using parameters in a class (Python). External processes can perform parameter operations via parameter services that are created by default when a node is instantiated. The services that are created by default are:

- `/node_name/describe_parameters`: Uses a service type of `rcl_interfaces/srv/DescribeParameters`. Given a list of parameter names, returns a list of descriptors associated with the parameters.

- `/node_name/get_parameter_types`: Uses a service type of `rcl_interfaces/srv/GetParameterTypes`. Given a list of parameter names, returns a list of parameter types associated with the parameters.

- `/node_name/get_parameters`: Uses a service type of `rcl_interfaces/srv/GetParameters`. Given a list of parameter names, returns a list of parameter values associated with the parameters.

- `/node_name/list_parameters`: Uses a service type of `rcl_interfaces/srv/ListParameters`. Given an optional list of parameter prefixes, returns a list of the available parameters with that prefix. If the prefixes are empty, returns all parameters.

- `/node_name/set_parameters`: Uses a service type of `rcl_interfaces/srv/SetParameters`. Given a list of parameter names and values, attempts to set the parameters on the node. Returns a list of results from trying to set each parameter; some of them may have succeeded and some may have failed.

- `/node_name/set_parameters_atomically`: Uses a service type of `rcl_interfaces/srv/SetParametersAtomically`. Given a list of parameter names and values, attempts to set the parameters on the node. Returns a single result from trying to set all parameters, so if one failed, all of them failed.

### Setting initial parameter values when running a node <a name="paramsetr"></a>
Initial parameter values can be set when running the node either through individual command-line arguments, or through YAML files. See Setting parameters directly from the command line for examples on how to set initial parameter values.

### Setting initial parameter values when launching nodes <a name="paramsetl"></a>
Initial parameter values can also be set when running the node through the ROS 2 launch facility. See this document for information on how to specify parameters via launch.

### Manipulating parameter values at runtime <a name="parammani"></a>
The `ros2 param` command is the general way to interact with parameters for nodes that are already running. `ros2 param` uses the parameter service API as described above to perform the various operations. See this how-to guide for details on how to use `ros2 param`.

### Migrating from ROS 1 <a name="parammig"></a>
The [Launch file migration guide](https://docs.ros.org/en/jazzy/How-To-Guides/Migrating-from-ROS1/Migrating-Launch-Files.html) explains how to migrate `param` and `rosparam` launch tags from ROS 1 to ROS 2.

The [YAML parameter file migration guide](https://docs.ros.org/en/jazzy/How-To-Guides/Migrating-from-ROS1/Migrating-Parameters.html) explains how to migrate parameter files from ROS 1 to ROS 2.

In ROS 1, the `roscore` acted like a global parameter blackboard where all nodes could get and set parameters. Since there is no central `roscore` in ROS 2, that functionality no longer exists. The recommended approach in ROS 2 is to use per-node parameters that are closely tied to the nodes that use them. If a global blackboard is still needed, it is possible to create a dedicated node for this purpose. ROS 2 ships with one in the `ros-jazzy-demo-nodes-cpp` package called `parameter_blackboard`; it can be run with:
```
ros2 run demo_nodes_cpp parameter_blackboard
```
The code for the `parameter_blackboard` is here.

### Introspection with command line tools <a name="intros"></a>
### Table of Contents
- [Usage](#intusage)
- [Example](#integ)
- [Behind the Scenes](#intbts)
- [Implementation](#intimp)

ROS 2 includes a suite of command-line tools for introspecting a ROS 2 system.

### Usage <a name="intusage"></a>
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

### Example <a name="integ"></a>

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
### Behind the scenes <a name="intbts"></a>

ROS 2 uses a distributed discovery process for nodes to connect to each other. As this process purposefully does not use a centralized discovery mechanism, it can take time for ROS nodes to discover all other participants in the ROS graph. Because of this, there is a long-running daemon in the background that stores information about the ROS graph to provide faster responses to queries, e.g. the list of node names.

The daemon is automatically started when the relevant command-line tools are used for the first time. You can run `ros2 daemon --help` for more options for interacting with the daemon.

### Implementation <a name="intimp"></a>

The source code for the `ros2 command` is available at https://github.com/ros2/ros2cli.

The `ros2` tool has been implemented as a framework that can be extended via plugins. For example, the [sros2](https://github.com/ros2/sros2) package provides a `security` sub-command that is automatically detected by the `ros2` tool if the `sros2` package is installed.

### Launch <a name="launch"></a>
A ROS 2 system typically consists of many nodes running across many different processes (and even different machines). While it is possible to run each of these nodes separately, it gets cumbersome quite quickly.

The launch system in ROS 2 is meant to automate the running of many nodes with a single command. It helps the user describe the configuration of their system and then executes it as described. The configuration of the system includes what programs to run, where to run them, what arguments to pass them, and ROS-specific conventions which make it easy to reuse components throughout the system by giving them each a different configuration. It is also responsible for monitoring the state of the processes launched, and reporting and/or reacting to changes in the state of those processes.

All of the above is specified in a launch file, which can be written in Python, XML, or YAML. This launch file can then be run using the `ros2 launch` command, and all of the nodes specified will be run.

The [design document](https://design.ros2.org/articles/roslaunch.html) details the goal of the design of ROS 2’s launch system (not all functionality is currently available).

### Client Libraries <a name="clilib"></a>
### Table of Contents

- [Overview](#cliove)
- [Supported Client Libraries](#clispt)
  - [The rclcpp Package](#clicpp)
  - [The rclpy Package](#clipy)
- [Common Functionality: rcl](#clicom)
- [Language-specific Functionality](#clilang)
- [Comparison to ROS 1](#clicomp)
- [Summary](#clisum)

### Overview <a name="cliove"></a>
Client libraries are the APIs that allow users to implement their ROS 2 code. Using client libraries, users gain access to ROS 2 concepts such as nodes, topics, services, etc. Client libraries come in a variety of programming languages so that users may write ROS 2 code in the language that is best-suited for their application. For example, you might prefer to write visualization tools in Python because it makes prototyping iterations faster, while for parts of your system that are concerned with efficiency, the nodes might be better implemented in C++.

Nodes written using different client libraries are able to share messages with each other because all client libraries implement code generators that provide users with the capability to interact with ROS 2 interface files in the respective language.

In addition to the language-specific communication tools, client libraries expose to users the core functionality that makes ROS “ROS”. For example, here is a list of functionality that can typically be accessed through a client library:

- Names and namespaces
- Time (real or simulated)
- Parameters
- Console logging
- Threading model
- Intra-process communication

### Supported Client Libraries <a name="clispt"></a>
The C++ client library (`rclcpp`) and the Python client library (`rclpy`) are both client libraries which utilize common functionality in rcl.

### The `rclcpp` package <a name="clicpp"></a>
The ROS Client Library for C++ (`rclcpp`) is the user facing, C++ idiomatic interface which provides all of the ROS client functionality like creating nodes, publishers, and subscriptions. `rclcpp` builds on top of `rcl` and the `rosidl` API, and it is designed to be used with the C++ messages generated by `rosidl_generator_cpp`.

`rclcpp` makes use of all the features of C++ and C++17 to make the interface as easy to use as possible, but since it reuses the implementation in `rcl` it is able maintain a consistent behavior with the other client libraries that use the `rcl` API.

The `rclcpp` repository is located on GitHub at ros2/rclcpp and contains the package `rclcpp`. The generated API documentation is here:

[api/rclcpp/index.html](http://docs.ros.org/en/jazzy/p/rclcpp)

### The `rclpy` package <a name="clipy"></a>
The ROS Client Library for Python (rclpy) is the Python counterpart to the C++ client library. Like the C++ client library, rclpy also builds on top of the rcl C API for its implementation. The interface provides an idiomatic Python experience that uses native Python types and patterns like lists and context objects. By using the rcl API in the implementation, it stays consistent with the other client libraries in terms of feature parity and behavior. In addition to providing Python idiomatic bindings around the rcl API and Python classes for each message, the Python client library takes care of the execution model, using threading.Thread or similar to run the functions in the rcl API.

Like C++ it generates custom Python code for each ROS message that the user interacts with, but unlike C++ it eventually converts the native Python message object into the C version of the message. All operations happen on the Python version of the messages until they need to be passed into the rcl layer, at which point they are converted into the plain C version of the message so it can be passed into the rcl C API. This is avoided if possible when communicating between publishers and subscriptions in the same process to cut down on the conversion into and out of Python.

The rclpy repository is located on GitHub at ros2/rclpy and contains the package rclpy. The generated API documentation is here:
[api/rclpy/index.html](https://docs.ros.org/en/jazzy/p/rclpy/)

### Common functionality: `rcl` <a name="clicom"></a>
Most of the functionality found in a client library is not specific to the programming language of the client library. For example, the behavior of parameters and the logic of namespaces should ideally be the same across all programming languages. Because of this, rather than implementing the common functionality from scratch, client libraries make use of a common core ROS Client Library (RCL) interface that implements logic and behavior of ROS concepts that is not language-specific. As a result, client libraries only need to wrap the common functionality in the RCL with foreign function interfaces. This keeps client libraries thinner and easier to develop. For this reason the common RCL functionality is exposed with C interfaces as the C language is typically the easiest language for client libraries to wrap.

In addition to making the client libraries light-weight, an advantage of having the common core is that the behavior between languages is more consistent. If any changes are made to the logic/behavior of the functionality in the core RCL – namespaces, for example – all client libraries that use the RCL will have these changes reflected. Furthermore, having the common core means that maintaining multiple client libraries becomes less work when it comes to bug fixes.

### Language-specific functionality <a name="clilang"></a>
Client library concepts that require language-specific features/properties are not implemented in the RCL but instead are implemented in each client library. For example, threading models used by “spin” functions will have implementations that are specific to the language of the client library.

### Comparison to ROS 1 <a name="clicomp"></a>
In ROS 1, all client libraries are developed “from the ground up”. This allows for the ROS 1 Python client library to be implemented purely in Python, for example, which brings benefits of such as not needing to compile code. However, naming conventions and behaviors are not always consistent between client libraries, bug fixes have to be done in multiple places, and there is a lot of functionality that has only ever been implemented in one client library (e.g. UDPROS).

### Summary <a name="clisum"></a>
By utilizing the common core ROS client library, client libraries written in a variety of programming languages are easier to write and have more consistent behavior.

## Intermediate Concepts <a name="int"></a>
### The ROS_DOMAIN_ID <a name="rosdom"></a>
### Table of Contents
- [Overview](#interover)
- [Choosing a Domain ID (short version)](#domshort)
- [Choosing a Domain ID (long version)](#domlong)
  - [Platform-specific constraints](#pltconst)
  - [Participant constraints](#prtconst)
  
### Overview <a name="interover"></a>
As explained elsewhere, the default middleware that ROS 2 uses for communication is DDS. In DDS, the primary mechanism for having different logical networks share a physical network is known as the Domain ID. ROS 2 nodes on the same domain can freely discover and send messages to each other, while ROS 2 nodes on different domains cannot. All ROS 2 nodes use domain ID 0 by default. To avoid interference between different groups of computers running ROS 2 on the same network, a different domain ID should be set for each group.

### Chooseing  a domain ID(Short version) <a name="domshort"></a>
The text below explains the derivation of the range of domain IDs that should be used in ROS 2. To skip that background and just choose a safe number, simply choose a domain ID between 0 and 101, inclusive.

### Chooseing  a domain ID(Long version) <a name="domlong"></a>
The domain ID is used by DDS to compute the UDP ports that will be used for discovery and communication. See this article for details on how the ports are computed. Remembering our basic networking, the UDP port is an unsigned 16-bit integer. Thus, the highest port number that can be allocated is 65535. Doing some math with the formula in the article above, this means that the highest domain ID that can possibly be assigned is 232, while the lowest that can be assigned is 0.

### Platform-Specific Constraints <a name="pltconst"></a>
For maximum compatibility, some additional platform-specific constraints should be followed when choosing a domain ID. In particular, it is best to avoid allocating domain IDs in the operating system’s ephemeral port range. This avoids possible conflicts between the ports used by the ROS 2 nodes and other networking services on the computers.

**For Linux:**
By default, the Linux kernel uses ports 32768-60999 for ephemeral ports. This means that domain IDs 0-101 and 215-232 can be safely used without colliding with ephemeral ports. The ephemeral port range is configurable in Linux by setting custom values in `/proc/sys/net/ipv4/ip_local_port_range`. If a custom ephemeral port range is used, the above numbers may have to be adjusted accordingly.

### Participant Constraints <a name="prtconst"></a>

For each ROS 2 process running on a computer, one DDS “participant” is created. Since each DDS participant takes up two ports on the computer, running more than 120 ROS 2 processes on one computer may spill over into other domain IDs or the ephemeral ports.

To see why, consider the domain IDs 1 and 2.

- Domain ID 1 uses port 7650 and 7651 for multicast.
- Domain ID 2 uses port 7900 and 7901 for multicast.
- When creating the 1st process (zeroth participant) in domain ID 1, the ports 7660 and 7661 are used for unicast.
- When creating the 120th process (119th participant) in domain ID 1, the ports 7898 and 7899 are used for unicast.
- When creating the 121st process (120th participant) in domain ID 1, the ports 7900 and 7901 are used for unicast and overlap with domain ID 2.


If it is known that the computer will only ever be on a single domain ID at a time, and the domain ID is low enough, it is safe to create more ROS 2 processes than this.

When choosing a domain ID that is near the top of the range of platform-specific domain IDs, one additional constraint should be considered.

For instance, assume a Linux computer with a domain ID of 101:

- The zero’th ROS 2 process on the computer will connect to ports 32650, 32651, 32660, and 32661.
- The first ROS 2 process on the computer will connect to ports 32650, 32651, 32662, and 32663.
- The 53rd ROS 2 process on the computer will connect to ports 32650, 32651, 32766, and 32767.
- The 54th ROS 2 process on the computer will connect to ports 32650, 32651, 32768, and 32769, running into the ephemeral port range.


Thus the maximum number of processes that should be created when using domain ID 101 on Linux is 54. Similarly, the maximum number of processes that should be created when using domain ID 232 on Linux is 63, as the maximum port number is 65535.

The situation is similar on macOS and Windows, though the numbers are different. On macOS and Windows, when choosing a domain ID of 166 (the top of the range), the maximum number of ROS 2 processes that can be created on a computer before running into the ephemeral port range is 120.

### Different ROS 2 midddleware vendors <a name="interdif"></a>
### Table of Contents
- [Supported RMW implementation](#rmwimp)
- [Multiple RMW implementation](mrmwimp)
- [Default RMW implementation](drmwimp)

ROS 2 is built on top of DDS/RTPS as its middleware, which provides discovery, serialization and transportation. This article explains the motivation behind using DDS implementations, and/or the RTPS wire protocol of DDS, in detail. In summary, DDS is an end-to-end middleware that provides features which are relevant to ROS systems, such as distributed discovery (not centralized like in ROS 1) and control over different “Quality of Service” options for the transportation.

DDS is an industry standard which is implemented by a range of vendors, such as RTI’s Connext DDS, eProsima’s Fast DDS, Eclipse’s Cyclone DDS, or GurumNetworks’s GurumDDS. RTPS (a.k.a. DDSI-RTPS) is the wire protocol used by DDS to communicate over the network.

ROS 2 supports multiple DDS/RTPS implementations because it is not necessarily “one size fits all” when it comes to choosing a vendor/implementation. There are many factors you might consider while choosing a middleware implementation: logistical considerations like the license, or technical considerations like platform availability, or computation footprint. Vendors may provide more than one DDS or RTPS implementation targeted at meeting different needs. For example, RTI has a few variations of their Connext implementation that vary in purpose, like one that specifically targets microcontrollers and another which targets applications requiring special safety certifications (we only support their standard desktop version at this time).

In order to use a DDS/RTPS implementation with ROS 2, a “ROS Middleware interface” (a.k.a. `rmw` interface or just `rmw`) package needs to be created that implements the abstract ROS middleware interface using the DDS or RTPS implementation’s API and tools. It’s a lot of work to implement and maintain RMW packages for supporting DDS implementations, but supporting at least a few implementations is important for ensuring that the ROS 2 codebase is not tied to any one particular implementation, as users may wish to switch out implementations depending on their project’s needs.

### Supported RMW Implementations <a name="rmwimp"></a>

| Product Name                  | License                       | RMW Implementation    | Status                                                                    |
|-------------------------------|-------------------------------|-----------------------|---------------------------------------------------------------------------|
| eProsima Fast DDS             | Apache 2                      | rmw_fastrtps_cpp      | Full support. Default RMW. Packaged with binary releases.                 |
| Eclipse Cyclone DDS           | Eclipse Public License v2.0   | rmw_cyclonedds_cpp    | Full support. Packaged with binary releases.                              |
| RTI Connext DDS               | commercial, research          | rmw_connextdds        | Full support. Support included in binaries, but Connext installed separately. |
| GurumNetworks GurumDDS        | commercial                    | rmw_gurumdds_cpp      | Community support. Support included in binaries, but GurumDDS installed separately. |

### Multiple RMW Implementations <a name="mrmwimp"></a>

The ROS 2 binary releases for currently active distros have built-in support for several RMW implementations out of the box (Fast DDS, RTI Connext Pro, Eclipse Cyclone DDS, GurumNetworks GurumDDS). The default is Fast DDS, which works without any additional installation steps because we distribute it with our binary packages.

Other RMWs like Cyclone DDS, Connext or GurumDDS can be enabled by installing additional packages, but without having to rebuild anything or replace any existing packages.

A ROS 2 workspace that has been built from source may build and install multiple RMW implementations simultaneously. While the core ROS 2 code is being compiled, any RMW implementation that is found will be built if the relevant DDS/RTPS implementation has been installed properly and the relevant environment variables have been configured. For example, if the code for the RMW package for RTI Connext DDS is in the workspace, it will be built if an installation of RTI’s Connext Pro can also be found.

For many cases you will find that nodes using different RMW implementations are able to communicate, however this is not true under all circumstances. Here is a list of inter-vendor communication configurations that are not supported:
**Fast DDS <-> Connext**
- `WString` published by Fast DDS can’t be received correctly by Connext on macOS

**Connext <-> Cyclone DDS**
- does not support pub/sub communication for `WString`

### Default RMW Implementation <a name="drmwimp"></a>
If a ROS 2 workspace has multiple RMW implementations, Fast DDS is selected as the default RMW implementation if it is available. If the Fast DDS RMW implementation is not installed, the RMW implementation with the first RMW implementation identifier in alphabetical order will be used. The implementation identifier is the name of the ROS package that provides the RMW implementation, e.g. `rmw_cyclonedds_cpp`. For example, if both `rmw_cyclonedds_cpp` and `rmw_connextdds` ROS packages are installed, `rmw_connextdds` would be the default. If `rmw_fastrtps_cpp` is ever installed, it would be the default.

### Logging and Logger Configuration <a name="interlog"></a>
### Table of Contents

- [Overview](#logover)
- [Severity Level](#logsever)
- [APIs](#apis)
- [Configuration](#logconfig)
  - [Environment Variables](#logenv)
  - [Node Creation](#lognode)
- [Logging Subsystem Design](#loglsd)
  - [rcutils](#rcutils)
  - [rcl_logging_spdlog](#rclspdlog)
  - [rcl](#rcl)
  - [rclcpp](#rclcpp)
  - [rclpy](#rclpy)


### Overview <a name="logover"></a>
The logging subsystem in ROS 2 aims to deliver logging messages to a variety of targets, including:

1. To the console (if one is attached)

2. To log files on disk (if local storage is available)

3. To the `/rosout` topic on the ROS 2 network

By default, log messages in ROS 2 nodes will go out to the console (on stderr), to log files on disk, and to the `/rosout` topic on the ROS 2 network. All of the targets can be individually enabled or disabled on a per-node basis.

The rest of this document will go over some of the ideas behind the logging subsystem.

### Severity Level <a name="logsever"></a>

Log messages have a severity level associated with them: `DEBUG`, `INFO`, `WARN`, `ERROR` or `FATAL`, in ascending order.

A logger will only process log messages with severity at or higher than a specified level chosen for the logger.

Each node has a logger associated with it that automatically includes the node’s name and namespace. If the node’s name is externally remapped to something other than what is defined in the source code, it will be reflected in the logger name. Non-node loggers can also be created that use a specific name.

Logger names represent a hierarchy. If the level of a logger named “abc.def” is unset, it will defer to the level of its parent named “abc”, and if that level is also unset, the default logger level will be used. When the level of logger “abc” is changed, all of its descendants (e.g. “abc.def”, “abc.ghi.jkl”) will have their level impacted unless their level has been explicitly set.

### APIs <a name="apis"></a>
These are the APIs that end users of the ROS 2 logging infrastructure should use, split up by client library.
- `logger.{debug,info,warning,error,fatal}`: Output the given Python string to the logging infrastructure. The calls accept the following keyword arguments to control behavior:
  - `throttle_duration_sec`: If not `None`, the duration of the throttle interval in floating-point seconds.
  - `skip_first`: If `True`, output the message all but the first time this line is hit.
  - `once`: If `True`, only output the message the first time this line is hit.
- `rclpy.logging.set_logger_level`: Set the logging level for a particular logger name to the given severity level.
- `rclpy.logging.get_logger_effective_level`: Given a logger name, return the logger level (which may be unset).

### Configuration <a name="logconfig"></a>
Since `rclcpp` and `rclpy` use the same underlying logging infrastructure, the configuration options are the same.
### Environment Variables <a name="logenv"></a>
The following environment variables control some aspects of the ROS 2 loggers. For each of the environment settings, note that this is a process-wide setting, so it applies to all nodes in that process.

- `ROS_LOG_DIR`: Control the logging directory that is used for writing logging messages to disk (if that is enabled). If non-empty, use the exact directory as specified in this variable. If empty, use the contents of the `ROS_HOME` environment variable to construct a path of the form `$ROS_HOME/.log`. In all cases, the `~` character is expanded to the user’s HOME directory.
- `ROS_HOME`: Control the home directory that is used for various ROS files, including logging and config files. In the context of logging, this variable is used to construct a path to a directory for log files. If non-empty, use the contents of this variable for the `ROS_HOME` path. In all cases, the `~` character is expanded to the user’s HOME directory.
- `RCUTILS_LOGGING_USE_STDOUT`: Control what stream output messages go to. If this is unset or `0`, use stderr. If this is `1`, use stdout.
- `RCUTILS_LOGGING_BUFFERED_STREAM`: Control whether the logging stream (as configured in `RCUTILS_LOGGING_USE_STDOUT`) should be line buffered or unbuffered. If this is unset, use the default of the stream (generally line buffered for stdout, and unbuffered for stderr). If this is `0`, force the stream to be unbuffered. If this is `1`, force the stream to be line buffered.
- `RCUTILS_COLORIZED_OUTPUT`: Control whether colors are used when outputting messages. If unset, automatically determine based on the platform and whether the console is a TTY. If `0`, force disable using colors for output. If `1`, force enable using colors for output.
- `RCUTILS_CONSOLE_OUTPUT_FORMAT`: Control the fields that are output for each log message. The available fields are:
  - `{severity}`: The severity level.
  - `{name}`: The name of the logger (may be empty).
  - `{message}`: The log message (may be empty).
  - `{function_name}`: The function name this was called from (may be empty).
  - `{file_name}`: The file name this was called from (may be empty).
  - `{time}`: The time in seconds since the epoch.
  - `{time_as_nanoseconds}`: The time in nanoseconds since the epoch.
  - `{line_number}`: The line number this was called from (may be empty).

If no format is given, a default of `[{severity}] [{time}] [{name}]: {message}` is used.

### Node Creation <a name="lognode"></a>
When initializing a ROS 2 node, it is possible to control some aspects of the behavior via node options. Since these are per-node options, they can be set differently for different nodes even when the nodes are composed into a single process.

-`log_levels` - The log level to use for a component within this particular node. This can be set with the following: ros2 run demo_nodes_cpp talker --ros-args --log-level talker:=DEBUG

-`external_log_config_file` - The external file to use to configure the backend logger. If it is NULL, the default configuration will be used. Note that the format of this file is backend-specific (and is currently unimplemented for the default backend logger of spdlog). This can be set with the following: ros2 run demo_nodes_cpp talker --ros-args --log-config-file log-config.txt

-`log_stdout_disabled` - Whether to disable writing log messages to the console. This can be done with the following: ros2 run demo_nodes_cpp talker --ros-args --disable-stdout-logs

-`log_rosout_disabled` - Whether to disable writing log messages out to /rosout. This can significantly save on network bandwidth, but external observers will not be able to monitor logging. This can be done with the following: ros2 run demo_nodes_cpp talker --ros-args --disable-rosout-logs

-`log_ext_lib_disabled` - Whether to completely disable the use of an external logger. This may be faster in some cases, but means that logs will not be written to disk. This can be done with the following: ros2 run demo_nodes_cpp talker --ros-args --disable-external-lib-logs

### rcutils <a name="rcutils"></a>
`rcutils` has a logging implementation that can format log messages according to a certain format (see `Configuration` above), and output those log messages to a console. `rcutils` implements a complete logging solution, but allows higher-level components to insert themselves into the logging infrastructure in a dependency-injection model. This will become more evident when we talk about the rcl layer below.

Note that this is a per-process logging implementation, so anything that is configured at this level will affect the entire process, not just individual nodes.

### rcl_logging_spdlog <a name="rclspdlog"></a>
`rcl_logging_spdlog` implements the `rcl_logging_interface` API, and thus provides external logging services to the `rcl` layer. In particular, the `rcl_logging_spdlog` implementation takes formatted log messages and writes them out to log files on disk using the `spdlog` library, typically within `~/.ros/log` (though this is configurable; see `Configuration` above).

### rcl <a name="rcl"></a>
The logging subsystem in `rcl` uses `rcutils` and `rcl_logging_spdlog` to provide the bulk of the ROS 2 logging services. When log messages come in, rcl decides where to send them. There are 3 main places that logging messages can be delivered; an individual node may have any combination of them enabled:

1. To the console via the `rcutils` layer

2. To disk via the `rcl_logging_spdlog` layer

3. To the `/rosout` topic on the ROS 2 network via the RMW layer

### rclcpp <a name="rclcpp"></a>
This is the main ROS 2 C++ API which sits atop the rcl API. In the context of logging, rclcpp provides the RCLCPP_ logging macros; see APIs above for a complete list. When one of the RCLCPP_ macros runs, it checks the current severity level of the node against the severity level of the macro. If the severity level of the macro is greater than or equal to the node severity level, the message will be formatted and output to all of the places that are currently configured. Note that rclcpp uses a global mutex for log calls, so all logging calls within the same process end up being single-threaded.

### rclpy <a name="rclpy"></a>
This is the main ROS 2 Python API which sits atop the `rcl` API. In the context of logging, `rclpy` provides the `logger.debug`-style functions; see `APIs` above for a complete list. When one of the `logger.debug` functions runs, it checks the current severity level of the node against the severity level of the macro. If the severity level of the macro is greater than or equal to the node severity level, the message will be formatted and output to all of the places that are currently configured.



## 8. Additional Reference Materials <a name="addref"></a>
### Books
1. "Programming Robots with ROS: A Practical Introduction to the Robot Operating System" by Morgan Quigley, Brian Gerkey, and William D. Smart
2. "ROS Robotics Projects: Build and control robots powered by the Robot Operating System, machine learning, and virtual reality" by Lentin Joseph
3. "ROS 2 Robot Programming: A Handbook for Robotics Software Developers" by Anis Koubaa
4. "Mastering ROS for Robotics Programming" by Lentin Joseph
5. "ROS Robotics by Example" by Carol Fairchild and Dr. Thomas L. Harman

### YouTube Channels
1. [The Construct: Robotics Education & Training](https://www.youtube.com/user/robotigniteacademy)
2. [ROS Developers Podcast](https://www.youtube.com/channel/UCt6Lag-vv25fTX3e11mVY1Q)
3. [ROSCon: ROS Conference Videos](https://www.youtube.com/user/roscon2013)
4. [Robotic Systems Lab ETH Zurich](https://www.youtube.com/user/RoboticSystemsLab)
5. [Isaac SDK & Robotics](https://www.youtube.com/channel/UCZSN8mgzSsiLHRgsWKuEALg)

### Courses
1. ROS Developer Learning Path on [Udemy](https://www.udemy.com/)
2. ROS for Beginners on [Coursera](https://www.coursera.org/)
3. [ROS Navigation in 5 Days](https://www.theconstructsim.com/ros-navigation-in-5-days/) by The Construct
4. ROS Industrial Training on [ROS-Industrial Consortium Americas](https://rosindustrial.org/)
5. ROS2 Tutorials on [ROS2 Documentation](https://docs.ros.org/)

### Forums and Communities
1. [ROS Discourse](https://discourse.ros.org/)
2. [ROS Answers](https://answers.ros.org/)
3. [ROS Discords](https://discord.com/invite/ros)
4. [ROS-Industrial Consortium](https://rosindustrial.org/)
5. ROS Meetup Groups (Search for local or virtual groups on [Meetup](https://www.meetup.com/) or similar platforms)

## 9. Project Assignment: Autonomous Delivery Robot <a name="oa"></a>

### Introduction:
In this project assignment, you will have the opportunity to design and develop an autonomous delivery robot using ROS 2, a powerful robotic middleware platform. The robot will be capable of navigating indoor environments and delivering items to predefined destinations autonomously. This assignment will allow you to apply your skills in robotics, software development, and system integration to tackle a real-world problem in logistics and automation.

### Objective:
Your objective is to design, develop, and demonstrate a fully functional autonomous delivery robot that can efficiently navigate indoor environments, avoid obstacles, and deliver items to specified locations. The robot should leverage ROS 2 for communication, control, and coordination between its various software modules and hardware components.

### Tasks:
1. **Hardware Selection and Integration:**
   - Research and select appropriate hardware components for the autonomous delivery robot, including sensors, actuators, and onboard computing hardware.
   - Assemble and integrate the selected hardware components into a functional robot platform.

2. **Software Development:**
   - Develop ROS 2 nodes for sensor data processing, perception, localization, path planning, and motion control.
   - Implement algorithms for environment mapping, obstacle detection, and navigation within ROS 2 framework.
   - Integrate software modules into a cohesive system architecture following ROS 2 best practices.

3. **Integration and Testing:**
   - Integrate the hardware and software components of the autonomous delivery robot.
   - Conduct comprehensive testing to ensure the reliability, safety, and performance of the robot in simulated and/or real-world environments.
   - Debug and troubleshoot any issues encountered during testing.

4. **User Interface Design:**
   - Design and implement a user-friendly interface for task assignment, monitoring, and interaction with the autonomous delivery robot.
   - Ensure seamless integration of the user interface with the ROS 2 control system.

5. **Documentation and Presentation:**
   - Document the design, implementation, and testing procedures of the autonomous delivery robot project.
   - Prepare a final presentation showcasing the robot's functionality, performance, and contributions to the field of autonomous robotics.
   - Present the project findings and demonstrate the robot to the stakeholders.

### Deliverables:
- Setup integration documentation.
- Software source code and ROS 2 package repository.
- Test reports and documentation of testing procedures.
- User interface design documentation and implementation.
- Final project presentation slides and demonstration video.

### Resources:
- ROS 2 documentation and tutorials.
- Robotics hardware components and platforms.
- Simulation environments such as Gazebo and RViz.
- Programming tools and libraries for robotics development.

**Note:** Regular progress updates and team meetings will be scheduled throughout the project timeline to track progress, address challenges, and provide support as needed. Collaboration and teamwork are essential for the successful completion of this project assignment.

## 12. Hands-on Exercises for Learning ROS 2 <a name="hoe"></a>

### Introduction to ROS 2
**Exercise:** Install ROS 2 on your preferred operating system (Ubuntu, Windows, macOS) and run the basic demo nodes to ensure successful installation.
**Note:** Ensure to follow the installation instructions carefully, and if you encounter any issues, refer to the official ROS 2 documentation or seek help from ROS communities.

### Setting Up the Environment
**Exercise:** Create a ROS 2 workspace, build a simple package, and run it to confirm that the workspace setup is correct.
**Note:** Pay attention to the workspace directory structure and use the appropriate build commands (e.g., `colcon build`) to compile your package.

### Basic Concepts
**Exercise:** Write a simple ROS 2 publisher node in both C++ and Python that publishes messages on a custom topic.
**Note:** Experiment with different message types and understand how to define message structures and publish data.

### ROS 2 Workspaces and Packages
**Exercise:** Create a new ROS 2 package with dependencies, build it within a workspace, and launch nodes from the package using launch files.
**Note:** Practice managing package dependencies and organizing your workspace efficiently.

### Programming with ROS 2
**Exercise:** Develop a ROS 2 service node that provides a basic service, and write a client node to interact with the service.
**Note:** Pay attention to the service definition, request-response communication, and error handling in client-server interactions.

### Advanced ROS 2 Features
**Exercise:** Integrate a simulated sensor (e.g., Lidar or camera) into a ROS 2 environment, and visualize its data in a visualization tool like RViz.
**Note:** Experiment with different sensor models and understand how to interface simulated hardware with ROS 2 nodes.

### Debugging and Testing
**Exercise:** Write unit tests for your ROS 2 nodes using testing frameworks like `gtest` or `pytest`, and debug any issues encountered during testing.
**Note:** Practice writing test cases to cover different scenarios and ensure the robustness of your code.

### Project: Building a Simple Robot Application
**Exercise:** Design and implement a simple robot application using ROS 2, incorporating concepts learned throughout the modules, such as node communication, services, and launch files.
**Note:** Focus on modular design and code reusability, and document your project structure and functionality effectively.
