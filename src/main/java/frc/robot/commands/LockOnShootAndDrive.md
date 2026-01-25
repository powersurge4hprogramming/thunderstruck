# The `LockOnShootAndDrive` Command: Turning our Entire Robot Into a Weapon System
This file is meant to explain, in detail, this complex custom command. I wanted to do this so that we can refer to it
for educational purposes, and for any other systems in the future that may take advantage of this kind of shooting.

## The `AimCamera`
The camera system is called *from here on out* as the `AimCamera`. This file,
[AimCamera](src/main/java/frc/robot/vision/AimCamera.java), is to simply give back the `x, y, z` distances of the
robot relative to the *AprilTag*. This package of data--the `x, y, z`--is called a
[Transform3d](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/geometry/Transform3d.html).
***So far, the only AprilTag we need to point at is the Hub's AprilTag(s).*** 

That's ok, we want to be pointing at the Hub's AprilTag when we are about to shoot. So, we are going to get that data
packet--the `Transform3d`--from the `AimCamera` with this method:
    [getHubRelativeLocation()](src/main/java/frc/robot/vision/AimCamera.java).
We are going to call it like this: `aimCamera.getHubRelativeLocation()`. You'd probably notice that you'd get a red line
under that code you copied. This is good. We need to *instanciate* the `AimCamera` system first. We do that like this:
```java
final AimCamera aimCamera = new AimCamera();
```

Now we have a real, *living*, `AimCamera`. Since we have created it--*instanciated it*--we can now **use it**. This is
very important. Now, we can combine these 2 lines:
```java
final AimCamera aimCamera = new AimCamera();
final Transform3d robotRelativeToHub = aimCamera.getHubRelativeLocation();
```

Fantastic! We have now the real life distance of the Hub relative to the robot: pretty amazing. Where do we go from here
now? We now need to give this data to the **shooting sub-system**.

## The `Shooter` Sub-System
The `Shooter` sub-system is the representation of the entire shooting mechanism. It's responsibility is to control the
***x number*** of motors that *drive* the shooting mechanism. The `Shooter`, though, needs to know how **hard** and at
**angle** to shoot the ball. Before we can shoot a ball, we first need to know how to shoot it. We need to understand
some [physics](src/main/java/frc/robot/physics).

## The Velocity and Angle Solving Function
We need to perform some [physics](src/main/java/frc/robot/physics). We need to get an angle and a velocity. This is done
with the [VelocityAngleSolver](src/main/java/frc/robot/physics/ballistics/VelocityAngleSolver.java). This *class* is
meant to solve, from our robot position relative to the Hub and our current x and y velocities, for our needed angle and
velocity to shoot the ball.

We must first *instantiate* a solver.
```java
final VelocityAngleSolver vASolver = new VelocityAngleSolver();
```
This is great, we now have some-"thing" to do our calculations. Let's tell it to do so. We need to give it our
`robotRelativeToHub`, `Transform3d` *object* so that it now knows how far we are from the Hub. Then, once it has that,
we can then give it our solver. We do it, like this:
```java
final VelocityAngleSolver vaSolver = new VelocityAngleSolver();
final ShotResult vaSolution = vaSolver.calculate(robotRelativeToHub, robotVx, robotVy, shapeScalar, isInBlockedMode);
```
As you can see, there is a lot more data this method, `calculate`, needs.

* `robotVx`: This is the velocity of the robot, in *m/s*, in a field relative *x* direction.
* `robotVy`: This is the velocity of the robot, in *m/s*, in a field relative *y* direction.
* `shapeScaler`: This is a dimensionless value that controls the shape of the shot curvature; for now, make it 0.8.
* `isInBlockedMode`: This is a boolean to control whether to not use the `shapeScalar`, and just shoot at a launch
    angle of 80 degress. As an aside, it certainly makes the math simpler.

All of this data we'll get to soon. For now, just make variables, with those names above the call to the `calculate()`
function. It'll be like, `double robotVx = 0;`, again and again. Do that just so that we can move on.

---
## Time to Make the Shot!
Ok, we now officially have what we need to make a shot! Let's now, make a command to the `Shooter` sub-system.