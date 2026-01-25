# Thunderstruck

## What Needs to Be Worked On Today
Today we are going to be *"hooking up"* (still trying to think of a better name for this) the camera system to the
shooter system.

---
The camera system is called *from here on out* as the `AimCamera`. This file
[Open main file](src/main/java/frc/robot/vision/AimCamera.java) is to simply give back the `x, y, z` distances of the
robot relative to the *AprilTag*. This package of data--the `x, y, z`--is called a
[Transform3d](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/geometry/Transform3d.html)
***So far, the only one we need to point at is the Hub's AprilTag(s).*** 

That's ok, we want to be pointing at the Hub's AprilTag when we are about to shoot. 

1. 

## What's Done
### Robot.java
I think **autonomous** and **teleop** will work here automatically. CTRE's generation software really hooked us up I
think.

## What Needs to Be Done

## What Needs to Be Tested

## Helpful Commands
`SwerveRequest.FieldCentricFacingAngle`
`SwerveRequest.FieldCentric`
`SwerveRequest.RobotCentric`