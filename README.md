# Thunderstruck
This is the code base for the **ThunderStruck** Robot. This file acts as our ***"work-board"*** to keep track of what to
do, to communicate remotely to each other, and allow for any of my review to happen at night outside of the shop.

## What Needs to Be Worked On Today

## Desmond
1. We will be continuing on button bindings.
2. We will also be figuring out binding behaviors.
    * When talking about these behaviors, we will be talking about program structures and even some algorithms.

## Mikey
We are going to need to get off of button binding. We don't know what to set, since there are things we still need to
decide.

So, since that is the case, we are going to move onto implementing some of the sub-sytems.

### The `Collector`
We are going to make a motor, and drive the motor from the input we made yesterday.

It will be important to refer to the [the `Shooter`](src/main/java/frc/robot/subsystems/Shooter.java) code when thinking
about, and implementing the `Collector` code.

---
#### The Setup
We are going to make a single motor **variable**. Will go in `The Collector's Parts` section of the file. The **type**
of motor we will be making is a `TalonFX`. We are going to call this **variable** `krakenX60`. We are not going to
*make* the motor here though. We are going to do that in the **Constructor**.

---
#### In the Constructor
We are going to make a `krakenX60` motor. **Assign** to our `krakenX60` a new `TalonFX`. In the `TalonFX()` function--
as an aside you are calling a function there... and it's a function called a constructor--pass to it a **CanBus** ID,
and a canbus to run on.

The ID you are going to pass to the `krakenX60` is going to come from
[the `CANBus` container](src/main/java/frc/robot/CANBus.java) where all of our relevent CANBUs settings will come from.
The first piece of data to the `TalonFX` **constructor** is an ID; so, lets give it that with this:
`CANBus.ID.COLLECTOR.MOTOR`. The second piece of data is *what* canbus it is on, and that comes from `CANBus.BUS.RIO`.

And there you go, you've made a motor :)

---
#### Let's Drive That Motor: run()
Here we are going to implement our code from before: our `run()` function. We have the `motorRPMScalar` from our
function's input. We are going to use that to set the RPM of the motor in the `run()` function.

We are going to write out code inside of our **"supplier**:
```java
return this.run(() -> {
    // Put Stuff here
});
```
What you write next goes in there.

We need to get the `double` from our `DoubleSupplier`, the `motorRPMScalar`, so that we can use it. We need a
`final double` **variable** to hold our motor rpm percentage (*scale*); so, make that varible and then assign to it the
value from our supplier by calling `get()` from our `motorRPMScalar`.

Then, `set()` the speed of our `krakenX60` motor by giving it the value of our previously created **variable**.

There ya go, you've implemented the `Collector`'s `run()` function!

---
---
## What's Done
### Robot.java
I think **autonomous** and **teleop** will work here automatically. CTRE's generation software really hooked us up I
think.

### In LockOnShootAndDrive.java
* **"Weapon Swapping"**! This command structure should be done.
    * More data needs to be provided soon.
* Figure out how to get the swerve drive's current x and y velocities.
    * <mark>The answer for this is in
        [this chat](https://grok.com/share/c2hhcmQtMg_bed28fc3-0692-43fb-815a-ccc28d2ea236)</mark>

## What Needs to Be Done
* Implement `LockOnShootAndDrive::execute`.
    * Figure out how to use `SwerveRequest.FieldCentricFacingAngle`.
* Implement `Shooter::setRPM`.
* Implement `Shooter::setLaunchAngle`.
* Implement `Shooter::manualShootBall` command.
* Implement `Collector::run` command.
* Implement `Collector::stop` command.
* Implement `Climber::upward` command.
    * Still waiting on what this would even be via the rest of the engineering team.
* Implement `Climber::downward` command.
    * Still waiting on what this would even be via the rest of the engineering team.
* Figure out how to get rumble working on `Triggers`.
* <mark>The entire autonomous plan!!!</mark>
    * Should we just handmake the plan? Probably not! Need to figure out how PathPlanner works!!!!
* Bind `Shooter::manualShootBall` command.
* Bind `Collector::run` command.
* Bind `Collector::stop` command.
* Bind `Climber::upward` command.
* Bind `Climber::downward` command.

## What Needs to Be Tested
* The `LockOnShootAndDrive` **y** binding.
* The `AimCamera::getHubRelativeLocation` function.

## Mentor Mike's Notes to Himself
`SwerveRequest.FieldCentricFacingAngle`

`SwerveRequest.FieldCentric`

`SwerveRequest.RobotCentric`

The `Collector` should definitely run on the roborio can bus. The climber probably will too. Not sure yet though on that
one.

CommandScheduler.getInstance().getActiveButtonLoop().clear();