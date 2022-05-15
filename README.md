# Miniproject: Robocop
Repo containing the source code for our robotics miniproject (MICRO-315), on which we worked on during the 2022 spring semester at EPFL.

We decided to create a radar/police car: our robot detects a noisy object moving above a certain speed, and starts chasing it after having activated sirens and blinking LEDs.

This code was made for an [epuck-2](https://www.gctronic.com/doc/index.php/e-puck2) with its [library](https://www.gctronic.com/doc/index.php?title=e-puck2_robot_side_development).

### User Manual
Reset the robot in front of a reference, ideally white colored, and do nothing for 2 seconds in order to let the robot calibrate. That's it!
As soon as a noisy object with speed above 7 cm/s passes, the robot will start chasing it. The object must be producing a sound, ideally sinusoidal, without which the robot cannot chase it.

### Compilation options
Our code contains several defines which change the behavior of the robot:
- `OBJECT_LENGTH` in _radar.c_: the program detects an object with length contained in this define. By default, it is set at 7.3 cm, i.e the width of an epuck-2.
- `MAX_SPEED` in _radar.c_: define that contains the speed limit. Without this define, the robot no longer detects an object using its speed in cm/s, and uses instead the number of measures given by the sensor per second. By default, this value is defined at 7 cm/s.
- `COMPUTE_SIGNED_ANGLE` in _audio_processing.c_: with this define, the robot can measure the angle between itself and a noise source, regardless where it is placed. The output produced is an angle between -180 and 180 degrees. Without this define, the robot can only measure the angle of a noise source in front of it, producing an output between 0 and 180 degrees. By default, `COMPUTE_SIGNED_ANGLE` is not defined.
