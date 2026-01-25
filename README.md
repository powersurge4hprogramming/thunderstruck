# Thunderstruck

## What Needs to Be Worked On Today
Today we are going to be *"hooking up"* (still trying to think of a better name for this) the camera system to the
shooter system.

### The `AimCamera`
The camera system is called *from here on out* as the `AimCamera`. This file,
[AimCamera](src/main/java/frc/robot/vision/AimCamera.java), is to simply give back the `x, y, z` distances of the
robot relative to the *AprilTag*. This package of data--the `x, y, z`--is called a
[Transform3d](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/geometry/Transform3d.html).
***So far, the only AprilTag we need to point at is the Hub's AprilTag(s).*** 

That's ok, we want to be pointing at the Hub's AprilTag when we are about to shoot. So, we are going to get that data
packet--the `Transform3d`--from the `AimCamera` with this method:
    [getHubRelativeLocation()](src/main/java/frc/robot/vision/AimCamera.java).
We are going to call it like this: `aimCamera.getHubRelativeLocation()`. Very nice, you'll probably notice that you'd
get a red line under that code you copied. This is good. We need to *instanciate* the `AimCamera` system first. We do
that like this:
```java
final AimCamera aimCamera = new AimCamera();
```

Now we have a real, *living*, `AimCamera`. Since we have created it*--instanciated it--*we can now **use it**. This is
very important. Now, we can combine these 2 lines:
```java
final AimCamera aimCamera = new AimCamera();
final Transform3d robotRelativeToHub = aimCamera.getHubRelativeLocation();
```

Fantastic! We have now the real life distance of the Hub relative to the robot: pretty amazing. Where do we go from here
now? We now need to give this data to the **shooting sub-system**.

### The `Shooter` Sub-System
The `Shooter` sub-system is the representation of the entire shooting mechanism. It's responsibility is to control the
***x number*** of motors that *drive* the shooting mechanism. The `Shooter`, though, needs to know how **hard** and at
**angle** to shoot the ball. Before we can shoot a ball, we first need to know how to shoot it. We need to understand
some [physics](src/main/java/frc/robot/physics).

We need to perform some [physics](src/main/java/frc/robot/physics). We need to get an angle and a velocity. This is done
with the [VelocityAngleSolver](src/main/java/frc/robot/physics/ballistics/VelocityAngleSolver.java). This

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