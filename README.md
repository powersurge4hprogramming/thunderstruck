# Thunderstruck
This is the code base for the **ThunderStruck** Robot. This file acts as our ***"work-board"*** to keep track of what to
do, to communicate remotely to each other, and allow for any of my review to happen at night outside of the shop.

## What Needs to Be Worked On Today
### The `LockOnShootAndDrive` Command
Today we are going to be *"hooking up"* (still trying to think of a better name for this) the
[`LockOnShootAndDrive`](src/main/java/frc/robot/commands/LockOnShootAndDrive.java) command to the controller.

### Requirements for the `LockOnShootAndDrive` Command
This command for the robot needs certain data and live functions (a.k.a *Suppliers*) passed--or given--to it. Before
we can even make our needed command, we must first gather the required components: these components are known as the
command's ***dependencies***.

1. `Shooter`: The command needs access to the shooter sub-system so that it can take complete control over it.
2. `CommandSwerveDrivetrain`: The command needs access to the swerve drivetrain so that it can control its yaw
    automatically and read its current x and y velocity while simultaneously allowing driver input on the x and y plane.
3. `AimCamera`: This is not a sub-system, but can the thought of as an entirely separate *system*. We only read data
    from the `AimCamera`, and therefore do not control it. We still need access to this live data though.
4. `DoubleSupplier xMove`: This will be used to drive the robot, field-centrically, on the x-axis. This is just a
    function that the command can run when it needs to get the user input from the controller. 
5. `DoubleSupplier yMove`: This will be used to drive the robot, field-centrically, on the y-axis. This is just a
    function that the command can run when it needs to get the user input from the controller. 

All of these things (*objects*) need to be created **before** we try to make (*instanciate*) our `LockOnShootAndDrive`
command.

#### How to Gather the Requirements
All of your work is going to be happening in [`RobotSystem`](src/main/java/frc/robot/RobotSystem.java). I have labelled
the various sections in the `RobotSystem` where things will go. Please use that as a guide.

---
##### The `CommandSwerveDrivetrain`
We already have a drivetrain sub-system defined in the `RobotSystem` and is a `class` called `CommandSwerveDrivetrain`
and is named `drivetrain`. So, that requirement is met already. We now need a sub-system to represent our `Shooter`.

---
##### The `Shooter`
The shooter is represented by the `class` called [`Shooter`](src/main/java/frc/robot/subsystems/Shooter.java). We need
to *define* and then *instanciate* the `Shooter` in the `RobotSystem`. This should be done in the ***Sub-Systems***
section. We first *define* our `Shooter` by telling the roborio to make space in its memory for the `Shooter` like this:

```java
private final Shooter shooter;
```

You may notice, at this point, that you have a red line. We must now `import` our `Shooter` code into the `RobotSystem`
with this line `import frc.robot.subsystems.Shooter;` and put it at the top of the file
**with the other `import frc.robot` lines**. Now that that error is gone, we can move on to *instanciating*, or
creating, our `Shooter`.

We must assign a created `Shooter` to our new `shooter` variable. We will do it all on one line now: the additional code
here is the *instanciation*.

```java
private final Shooter shooter = new Shooter();
```

We now, officially, have a `Shooter` sub-system that we can actually use! That is only one requirement though. Let's
move onto the rest.

---
##### The `AimCamera`
We can now do the same thing with the `AimCamera`. I will not give the same explanation I did with the `Shooter`
sub-system as to what we are doing, I would only be repeating myself.

Let's first import the `AimCamera` into our `RobotSystem` class: `import frc.robot.vision.AimCamera;`; and please put it
with the other related `import` statements.

Next we will *define* and *instanciate* in the same statement like we did with the `Shooter`:

```java
private final AimCamera aimCamera = new AimCamera();
```

Put that with out other sub-systems. Here come's the next requirement.

---
##### The `DoubleSupplier`'s
These we will be directly providing to the `LockOnShootAndDrive` command. We will not predefine those. We will now move
onto setting up the `Trigger` and the `LockOnShootAndDrive` command.

---
### Mapping the `LockOnShootAndDrive` Command to a Button
Remember how we talked about the `Trigger` that will, once a condition has been met, will *trigger* a command that we
give it? That's what we are going to setup now. For now, until someone--like the driver--tells us otherwise, I want the
*trigger* for our command to be the **y** button. Think of it like we are swapping weapons: we are swapping from manual
aim to auto-aim.

You'll notice from what is already in the `configureBindings()` method that we have no `Trigger` obvious *objects*. This
is on purpose. We can use the default `Trigger` that the `CommandXboxController` class provides us: `y()`. This method
returns a `Trigger`, the thing we need, that is attached to the **y** button. This is great because we didn't have to
make our own `Trigger`. So, the default `y()` *trigger* is what we will use. Let's get this `Trigger`:

```java
controller.y();
```

Simple, right! Now, we want to toggle on and off our *"weapon"* of choice. We will use another method on our current
`y()` `Trigger` that returns the same `y()` `Trigger`, but we tell it ***on what condition*** and
***what we want it to execute***.

Since we are using the **y** button as a toggle right now, we want it to act as one. We have a pre-made condition that
the `Trigger` class provides to us: it is called `toggleOnTrue()`. This *conditional trigger* will start the given
command if the command is not already scheduled, or cancel the given command if it is scheduled. This is the exact
behavior we want! All we have to do is provide it the command we want. First though, let's see how our code will look
right now:

```java
controller.y().toggleOnTrue();
```

Let's now give the `toggleOnTrue()` *trigger* our `LockOneShootAndDrive` command:

```java
controller.y().toggleOnTrue(
                new LockOnShootAndDrive(
                                null,
                                null,
                                null,
                                null,
                                null));
```
Please change the `null`s to their appropriate values. The last 2 I will provide:

```java
controller.y().toggleOnTrue(
                new LockOnShootAndDrive(
                                null,
                                drivetrain,
                                null,
                                () -> -controller.getLeftX() * MaxSpeed,
                                () -> -controller.getLeftY() * MaxSpeed));
```
This syntax is odd, and I don't want to go over it right now. I'll explain another day.

Once you change the `null`s, you have completed today's task! Fantastic job!!

---
---
## What's Done
### Robot.java
I think **autonomous** and **teleop** will work here automatically. CTRE's generation software really hooked us up I
think.

## What Needs to Be Done

## What Needs to Be Tested

## Helpful Commands for Mentor Mike
`SwerveRequest.FieldCentricFacingAngle`
`SwerveRequest.FieldCentric`
`SwerveRequest.RobotCentric`
