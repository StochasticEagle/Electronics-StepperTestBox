# Stepper Motor Control Test Box

![Alt text](Images.png?raw=true "Operation")

### About

This test box allows one to control and test stepper motors in or out of a system, and to test system mechanics without having control boards installed or complete systems.  The built-in LED on pin13 is used as a heartbeat LED, flashing with a blink rate of 1 Hz.  No serial data is sent over usb, and the USB connection is not used in the device.  Loop timing on the 16 MHz Arduino is about 500 us, so it can't do high speeds with more microsteps (like 1:16) and may skip.

### Controls

* Green - "Run" button, tells the controller to issue pulses to the stepper driver
* Red - "Stop" button, stops the motor
* Black (short press) - "Direction", changes the direction of rotation for the motor
* Black (long press) - "Microsteps", changes the microstepping for the motor driver
* Knob - "Speed", changes the speed from 0 to 100% (whole numbers, 0 to 300 RPM at 1:1 microstepping)

### Output

Motor will rotate, put a flag on it to see direction and speed easily
LCD screen shows microstep setting, speed percent, number of steps (n), and direction of rotation.
Backlight colors:
* Green for "run"
* Red for "stop"
* Yellow for "run" if speed is 0%.

LCD screen sample:

  ```
  uSteps: 1/16     67%
  n:        722345 r:o
  ```

### Compiling Software

This program may be built and uploaded to Arduino Nano using PlatformIO or Arduino (tested in both).  PlatformIO was used with the -O3 option (because Arduino IDE lacks this) to optimize the code for speed prior to upload.

```
RAM:   [==        ]  15.1% (used 309 bytes from 2048 bytes)
Flash: [===       ]  27.1% (used 8336 bytes from 30720 bytes)
```

### Parts

* Arduino Nano
* Stepper motor driver
* Pin headers
* 16x2 character LCD (RGB backlight, otherwise use RGB LED with circuit modifications)
* Buttons
* Potentiometer with knob (10 K ohm)
* Trimmer potentionmeter (10 K ohm, small)
* 33 ohm resistor (red LED current balancing, others may or may not be needed depending on LED setup)
* Logic Power Supply:
    * LM7805, heatsink, Capacitors (100 uF / 50 WVDC, 10 uF, and 10 nF ("104" ceramic pancake))
    * Or adjustable DC-DC voltage regulator setup with 100 uF / 50 WVDC input capacitor
* Wires
* Prototyping boards or custom PCB
* Project box (optional, breadboard use possible)
* 9V to 30V DC Power Supply (for motor power and voltage regulator supply)


### Circuit Diagram

![Alt text](CircuitDiagram.png?raw=true "Circuit Diagram")

