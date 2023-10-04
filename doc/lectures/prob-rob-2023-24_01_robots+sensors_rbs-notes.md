# Prob-Rob-2020-21_Robots+Sensores (RBS Notes)

_(2023/09/26)_

- [Slides](/doc/lectures/prob-rob-2023-24_01_robots+sensors.pdf) (2023/2024)
- [YouTube Video](https://www.youtube.com/watch?v=tgFW4y0TY6c) (2020/2021)

## Egomotion Sensors

Motion of the device estimated by the reference frame of the device itself.

- Wheel encoders: measure the revolutions count / location of the shaft
  - optical encoders: light pass through the holes, square signals allow count
    of encoder ticks
  - magnetic: hall-effect
- Inertial Measurement Units (IMU)
  - Accelerometers
  - Gyros
  - Android app to see sensors data from your phone:
    [Device Info](https://play.google.com/store/apps/details?id=com.liuzh.deviceinfo)

Integration results also integrate the error >>> specially in constant bias,
the error increases incrementally...

Only with egomotion not possible to localize the robot for a long time... In
general, we need external reference systems / observations to correct the error
of the egomtion.

## Measuring the environment

- Active: inject energy into the environment > measurement the return of the
  energy
  - Ultrasound
  - Laser range finder
  - Strucutred-light cameras
  - Infrared
- Passive: meassure the energy of the environment itself
  - RGB Cameras
  - Tactiles

### Sonars: Sound Navigation and Ranging

- Device that emits a sound, and measures the echo. However, the sound is not
  very directional specially in the air (in water is a lot more directional!).
- Sound disperses according to a conic form.
- To measure ~300m > sound speed ~343m/s ~ 0.5Hz maximum frequency reading (note
  that the device send the signal + waits for its return!)

### Laser Scanner

- Beam of pulsing light, send / emit the light and waits for the return signal
  (emitter + receiver)
- if the mirror inside the sensor tilts, we can have a 3D scanner
- are approved in terms of security standards for collision detection
  - Giorgio also has the Ouster 128 beams (it seems)

### RGB Monocular Camera

- 2D projection of a 3D structure + light intesity of each pixel
  - pin-hole camera: tends to curve straight lines in monocular camera
    (specially more in fish hole caameras)
- exposure time: how long the image is being capture
  - more time may blurred the image
  - the more time shutter open the slower we have to move
  - the less time, less light may be captured
- image sensitivity
- with only single camera, the depth of the object is not observable!
  - see Structure from Motion!

### RGB Stereo Camera

- distance ~~triangulation
- epipolar geometry
- textureless environments >>> cannot determine the depth of the scene...

### RGB-D Camera

- projector to estimate the depth of the object
- stereoscopia versus Time-of-Flight

## Mobile Base

- sensors + actuators
- of couse, all mechanicaal structure, electricaal connections, etc.
- itnerface suitable for autonomous navigation
- differential drive robots are cheap, simpler, etc.

### MARRtino

- https://www.marrtino.org/
- goal is to anyone have low cost and accessible platforms

### Orazio

- simplified of complete redesign of MARRtino
