# OSCC-Apollo
The customised host-side OSCC package rewritten from [PolySync OSCC](https://github.com/PolySync/oscc). For easy integration with Apollo, we have swtiched to C++ and Bazel, KIA Niro is the default vehicle. Please reset the macros in "shared_variable.bzl" for KIA Soul or KIA Soul EV. 

## Role of this package

## Installation

## Demo: Joystick control
Initilise the CAN interface. 
```bash
# Set can0 for publishing commands
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# Set can1 for subscribe status
sudo ip link set can1 type can bitrate 500000
sudo ip link set up can1
```

Start the joystick controller on can0
```bash
bazel run //demo:niro_jscmd 0
```
In the meantitme, listen to the vehicle status on can1
```bash
bazel run //demo:niro_jscmd 1
```
