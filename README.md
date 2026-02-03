# Thunderstruck
This is the code base for the **ThunderStruck** Robot. This file acts as our ***"work-board"*** to keep track of what to
do, to communicate remotely to each other, and allow for any of my review to happen at night outside of the shop.

## What Needs to Be Worked On Today

## Desmond
1. We are going to do the camera glueing/taping again. I can't get the cap on for the lense securely.
    * Once we do this we will have to re-calibrate.
2. We will be continuing on button bindings.
3. We will also be figuring out binding behaviors.
    * When talking about these behaviors, we will be talking about program structures and even some algorithms.


## Quick mike notes from mikey
I had to import the SparkMax class, but it didn't work. is it something with how I imported it?
It's also angry at me for something in the `run()` function. something involving parentheses, but I'm not quite sure what.

### My response to you :smile:
If you run into any issues, just help out Hogan and the rest of the engineering team. I'm sure they'll need it. You
could wait until I get back to continue with the programming. It may be best if you continue to run into issues to just
wait.

To import the `SparkMax` class, you actually do this at the top:
```java
import com.revrobotics.spark.SparkMax;
```

The `SparkMax` instanciation needs to happen inside the `Collector()` function--the instanciation is the part that
happens after the `=` symbol (the assignment operator)--and where you put the code after the `=` is not correct either:
**which is ok, you're still learning.** What you are doing in `The Collector's Parts` section is defining what is a part
of the `Collector` sub-system. Look [at the `Shooter` class](src/main/java/frc/robot/subsystems/Shooter.java#L17) to see
how the motor should be **defined**. Plus, don't forget that *everything* is case sensitive.

Then, you'd still have to worry about creation (instanciation) inside of the `Collector()` function. That is where you
assign to the `neo` variable. You'd have to assign a new `SparkMax()` to the `neo` variable, and give to it the correct
data. The data you give to the `SparkMax` function is explained below. If anything, check out its documentation
[here](https://codedocs.revrobotics.com/java/com/revrobotics/spark/sparkmax#%3Cinit%3E(int,com.revrobotics.spark.SparkLowLevel.MotorType)).
That should hopefully clear some things up. The ID will come from our own
[our own `CANBus`](src/main/java/frc/robot/CANBus.java) and the `MotorType` 
[will come from here](https://codedocs.revrobotics.com/java/com/revrobotics/spark/sparklowlevel.motortype).
The ID is the first value to go to the `SparkMax()` function, and the `MotorType` is the second.

If this still doesn't clear anything up, then ***definitely wait for me*** to get back and focus on helping the rest of
the engineering team **build** some amazing sub-systems! You'll learn a ton--I'm sure--and plus you'll have a blast! If
anything, when I get back, we can tackle the code to bring everything to life since we'll have almost all--or maybe even
all--of our sub-systems physically created :smile: So, basically, don't worry if this is still very difficult. It's not
something to be worried about :smile:

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
We are going to make a single motor **variable** that will go in `The Collector's Parts` section of the file. The
**type** of motor we will be making is a `SparkMax`. We are going to call this **variable** `neo`. We are not going
to *make* the motor here though. We are going to do that in the **Constructor**.

---
#### In the Constructor
We are going to make a `neo` motor. **Assign** to our `neo` a new `SparkMax`. In the `SparkMax()` function--
as an aside you are calling a function there... and it's a function called a constructor--pass to it a **CanBus** ID,
and a `MotorType`.

The ID you are going to pass to the `neo` is going to come from
[the `CANBus` container](src/main/java/frc/robot/CANBus.java) where all of our relevent CANBus settings will come from.
The first piece of data to the `SparkMax` **constructor** is an ID; so, lets give it that with this:
`CANBus.ID.COLLECTOR.MOTOR`. Is the kind of motor it is, for us we'll set it to brushless.

And there you go, you've made a motor :smile:

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
`final double` **variable** to hold our motor rpm percentage (*scale*); so, make that variable and then assign to it the
value from our supplier by calling `get()` from our `motorRPMScalar`.

Then, `set()` the speed of our `neo` motor by giving it the value of our previously created **variable**.

There ya go, you've implemented the `Collector`'s `run()` function!

---
---
## What's Done
### Robot.java
I think **autonomous** and **teleop** will work here automatically. CTRE's generation software really hooked us up I
think.

### In LockOnShootAndDrive.java
* **"Weapon Swapping"**! This command structure should be done.
    * Actually, it may not work or be final.
* Figure out how to get the swerve drive's current x and y velocities.
    * <mark>The answer for this is in
        [this chat](https://grok.com/share/c2hhcmQtMg_bed28fc3-0692-43fb-815a-ccc28d2ea236)</mark>

## What Needs to Be Done
* Give data to the `Shooter::manualShootBall` command.
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
* Bind `Collector::run` command.
* Bind `Collector::stop` command.
* Bind `Climber::upward` command.
* Bind `Climber::downward` command.

## What Needs to Be Tested
* The `LockOnShootAndDrive` **y** binding.
* The `AimCamera::getHubRelativeLocation` function.
* The `LockOnShootAndDrive::execute` function.

## Mentor Mike's Notes to Himself
`SwerveRequest.FieldCentricFacingAngle`

`SwerveRequest.FieldCentric`

`SwerveRequest.RobotCentric`

The `Collector` should definitely run on the roborio can bus. The climber probably will too. Not sure yet though on that
one.

// 1. Clear the Commands so there are no ghost commands.
CommandScheduler.getInstance().cancelAll();
// 2. Clear the bindings
// This removes all Trigger/Button listeners from the active loop.
CommandScheduler.getInstance().getActiveButtonLoop().clear();

How to keep the profile swap going?
**Got it**! All of our Commands will go inside of an array, and then when we map, we grab them by reference and add them
onto a `Trigger`. That way, we can cancel selectively by iterating over the array, cancelling as we go, and skipping
over the cycleing commands.
