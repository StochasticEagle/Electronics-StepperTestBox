# Stepper Motor Control Test Box

### About

This test box allows one to control and test stepper motors in or out of a system, and to test system mechanics without having control boards installed or complete systems.

### Parts

* Arduino Nano
* Stepper motor driver
* Pin headers
* RGB LED (optional)
* 16x2 character LCD (RGB backlight preferred)
* Buttons
* Potentiometer with knob
* Wires
* Prototyping boards or custom PCB
* Project box (optional, breadboard use possible)
* 33 ohm resistor (red LED current balancing, others may or may not be needed depending on LED setup)
* LM7805 and capacitors or other voltage regulator setup
* 9V to 30V DC Power Supply

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
