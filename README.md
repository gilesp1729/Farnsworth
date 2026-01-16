# Farnsworth

### Control VESC motor by pedelec and display data via BLE

This version is for Bafang M510 and other mid motors that have an analog torque sensor and quadrature cadence sensors. 

### Scope

To allow gutted (motor only) Bafang mid motors to be controlled by an externally mounted VESC, and allow users to have full control. It addionally outputs speed, cadence and power data via the BLE Cycle Power service to popular cycling apps (such as SuperCycle), or full motor parameters to the Babelfish app for VESC.

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

### Works in progress

* Enforce the speed limit.

* Experiment with run-on time to get smooth running through brief pauses in pedalling.

* Display using Babelfish needs to be able to set and write the PAS level to Farnsworth, among other things.

* Electrical noise during motor startup (open-loop running of sensorless motor) is confusing the speed and cadence readings.

The schematics will be published when the noise-suppression stuff gets sorted out.

In the meantime here are some pictures of the  build.

![](assets/9783d6e2ae019c85c8a1d572a10ac323d78067f8.jpg)

The phase wires and the torque sensor/temp sensor wires come out of the top of the motor housing. 

![2025-12-16 16.35.41.jpg](assets/e424081b1db17b51ed1962ccfad85ceede3a8939.jpg)

Testing the motor with the Spintend Ubox 100V/100A Vesc. On the left is a 20 amp buck converter that takes 48V in from the battery and allows current limiting.

![2026-01-03 14.32.52.jpg](assets/35ad150fe3c49171f3f64a82bf32cf34c6c7a9bb.jpg)

The wires are led out on the left side, since it's the only place the gap is large enough without cutting the carbon fibre (which I was not going to do)

![2026-01-03 14.46.38.jpg](assets/906d35ebaf8a7321f2ec880c72e25255c60ecdcd.jpg)

The VESC and Arduino Nano 33BLE mounted on the 3D-printed cradle. Tis is shaped to fit the frame just below the shock and has a gap below for a small heatsink on the VESC.



![2026-01-03 15.00.54.jpg](assets/11b4f09d5982cb35fc8ddea7af2d9c92217d62c3.jpg)

Like this.



![2026-01-07 15.51.30.jpg](assets/af513780f12cba9293e3a753898ddfaa4e218646.jpg)

The lid is made in two halves so it can be slipped in under the shock.
