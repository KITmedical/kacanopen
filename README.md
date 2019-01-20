[![Licence](https://img.shields.io/badge/licence-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![drivers_lgpl Licence](https://img.shields.io/badge/drivers__lgpl%20licence-LGPL%202.1%2B-blue.svg)](https://opensource.org/licenses/LGPL-2.1)
[![Build Status](https://api.travis-ci.org/KITmedical/kacanopen.svg?branch=master)](https://travis-ci.org/KITmedical/kacanopen)

# KaCanOpen

KaCanOpen is an easy-to-use [CANopen](https://en.wikipedia.org/wiki/CANopen) stack, which consists of four parts:

* __Drivers:__ A wide range of hardware is supported using different CAN drivers. They have been developed by the [CanFestival project](http://www.canfestival.org/). Read [this](drivers_lgpl/README) for details.

* __Core:__ This is a library which implements basic CANopen protocols like [NMT](https://en.wikipedia.org/wiki/CANopen#Network_management_.28NMT.29_protocols), [SDO](https://en.wikipedia.org/wiki/CANopen#Service_Data_Object_.28SDO.29_protocol) and [PDO](https://en.wikipedia.org/wiki/CANopen#Process_Data_Object_.28PDO.29_protocol). As an example, you can easily fetch a value from a device (*uploading* in CANopen terminology) via `core.sdo.upload(node_id, index, subindex)`. It furthermore allows you to register callbacks on certain events or any incoming messages, so one can build arbitrary CANopen nodes (master or slave) using this library.

* __Master:__ This library is intended to be used for a master node. It detects all nodes in a network and allows to access them via standardized [CiA® profiles](http://www.can-cia.org/can-knowledge/canopen/canopen-profiles/). For example, on a motor device (profile CiA® 402) you can simply call `mymotor.set_entry("Target velocity", 500)`. A main feature of this library is transparent SDO/PDO access to dictionary entries: By default a value is fetched and set via SDO, but you can configure PDO mappings to instead keep the value up-to-date in background via (more lightweight) PDO messages. The call itself (`mymotor.set_entry("Target velocity", 500)`) keeps unchanged. The dictionary structure of a device can be loaded automatically from a set of generic, CiA® profile-specific, or device-specific [EDS](https://en.wikipedia.org/wiki/CANopen#Electronic_Data_Sheet) files which is distributed along with KaCanOpen. This allows you to write a meaningful Plug and Play master node with only a few lines of code.

* __ROS Bridge:__ This library provides a bridge to a [ROS](http://www.ros.org/) network, which makes KaCanOpen especially interesting for robotics. After setting up the CANopen network, the library can publish slave nodes so they are accessible through ROS messages. Special effort is put into the use of standardized message types which allows interaction with other software from the ROS universe. For example, motors can be operated using JointState messages.

KaCanOpen is designed to make use of modern C++11/14 features and to avoid redundant code on the user side wherever possible. This makes the interface neater than it is in other products on the market.

## Quick start

First make sure you've got a recent C++ compiler with C++14 support ([GCC](https://gcc.gnu.org/) >= 4.9, [Clang](http://clang.llvm.org/) >= 3.6), as well as [CMake](https://cmake.org/) >= 3.2 and [Boost](http://www.boost.org/) >= 1.46.1.

KaCanOpen without the ROS part can be built easily using CMake:

~~~bash
git clone https://github.com/KITmedical/kacanopen.git
cd kacanopen
mkdir build
cd build
cmake -DDRIVER=<driver> -DNO_ROS=On ..
make
~~~

`<driver>` can be one of the following: serial, socket, virtual, lincan, peak\_linux.

The `examples` directory lists some examples on how to use KaCanOpen libraries. You can run them from the `build/examples` directory.

KaCanOpen including the ROS part must be built using [Catkin](http://wiki.ros.org/catkin/Tutorials). Make sure you have [ROS Jade](http://www.ros.org/install/) installed. Go to your [workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and clone the repository into `src`:

~~~bash
cd your_catkin_workspace/src
git clone https://github.com/KITmedical/kacanopen.git
cd ..
catkin_make -DDRIVER=<driver>
~~~

When building with Catkin, you can excute example programs like that:

~~~bash
cd your_catkin_workspace
source devel/setup.bash
rosrun kacanopen kacanopen_example_motor_and_io_bridge # roscore needs to be running
~~~

Complete build instructions can be found [here](doc/Installation.md).

## Examples

There are several examples has been implemented in the "example" folder.

* __master.cpp:__ This is an example which shows the usage of the Master library.

* __core.cpp:__ This is an example which shows the usage of the Core library.

* __pdo.cpp:__ This example runs a counter completely without SDO transfers.There must be a CiA 401 device which is configured to send 'Read input 8-bit/Digital Inputs 1-8'and 'Read input 8-bit/Digital Inputs 9-16' via TPDO1 and to receive 'Write output 8-bit/Digital Outputs 1-8' via RPDO1.

* __simple_sdo_rw.cpp:__ This example shows how to access a slave device with SDO read write. Alsmost all the device parameters can be read and or write by SDO transfer only. But this will not be time optimised as SDOs are not meant to be used for real time/ time critical parameters. A Roboteq CANOpen motor driver was used to test the program.

* __simple_pdo_rw.cpp:__ This example shows how to access a slave device with PDOs only. The slave device must be preconfigured to transmit TPDOs and receive RPDOs. Dynamic pdo mapping for the slave device has not been shown in this exmple, though this can be done by simple SDO transfer.A Roboteq CANOpen motor driver was used to test the program.

* __periodic_tpdo_write.cpp:__ This example shows how to use periodic PDO transfer by the master. A transmit pdo has been configured which will be received by the device rpdo1 periodically at every 250ms. A CANopenSocket simulated/emulated CiA401 slave.

* __simple_pdo_rw_dynamic_mapping.cpp:__ This example shows how to access a slave device with PDOs only. The slave device must be preconfigured to transmit TPDOs and receive RPDOs. Dynamic pdo mapping for the slave device has been implemented using SDO transfer.A CANopenSocket simulated/emulated CiA401 slave.

## Documentation

Full documentation can be found at [https://kitmedical.github.io/kacanopen/](https://kitmedical.github.io/kacanopen/).

## License

Core, Master and ROS Bridge are licensed under the [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause) license. Drivers from [CanFestival](http://www.canfestival.org/) are licensed under the [LGPLv2.1+](https://opensource.org/licenses/LGPL-2.1) license.
