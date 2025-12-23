# Farnsworth

### Control VESC motor by pedelec and display data via BLE

This version is for Bafang M510 and other mid motors that have an analog torque sensor and quadrature cadence sensors. 

### Scope

To allow gutted (motor only) bafang mid motors to be controlled by an externally mounted VESC, and allow users to have full control. It addionally outputs speed, cadence and power data via BLE to popular cycling apps (such as SuperCycle), or full motor parameters to the Babelfish app.

### Sensors used

* 6-pin analog torque/cadence sensor in M510 or M560 motor (the CAN bus ones are not supported)

* 2-pin temp sensor (there are no Hall sensors in these motors)

* Speed sensor (I used a Bosch one because it's much easier being only 2-wire)

### Runs on

Arduino Nano 33 BLE or BLE Sense.

### Library dependencies

* VESCUart

* ArduinoBLE

* RunningAverage
