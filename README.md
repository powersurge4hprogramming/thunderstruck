# Thunderstruck
This is the code base for the **ThunderStruck** Robot. This file acts as our ***"work-board"*** to keep track of what to
do, to communicate remotely to each other, and allow for any of my review to happen at night outside of the shop.

## What Needs to Be Worked On Today
### More Mappings
#### Actual "Weapon" Swapping
I kinda fibbed that yesterday's logic would work. We are only toggling `LockOnShootAndDrive` on and off; but what is the
roborio suppossed to run when it is toggled off? **The roborio has nothing to run when that toggle is off.** Obviously an
issue. So, we need to have our own toggling state to control manual vs. auto-aim. This will require our own `Trigger`.

First off we are going to need some kind of **auto-aim flag variable** to keep track of what "weapon" we have selected.
Define this with a initial value in the ***Driver Inputs*** code section.

Then, in the old **y()** toggle, we are going to change it to a "Command Orchestrator."

Here is the
[`Trigger`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/button/Trigger.html)
documentation and the
[`Commands`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Commands.html)
documentation.

First off, define our `LockOnShootAndDrive` outside the trigger so that is its own variable. Then, do the same with the
manual aim command from the `Shooter` class. These should be there own variables now inside of the `configureBindings()`
function now. These will be used in the next step.

Change the trigger from a toggle to a trigger that will execute when the button is true. Then, in that trigger, give it
a command from `Commands` to run one time. This onetime command will check if we are in lock-on mode. If we are, check
if the manual aim command `isScheduled()` and if it is, `cancel()` it. Then, `schedule()` the lock on command variable
from earlier and finally set the auto aim flag to `false`. Else if we are not in lock-on mode, then we check if the
lock-on command `isScheduled()` and if it is then `cancel()` it, then `schedule()` the manual aim command and set the
auto aim flag to `true`.

We now have actual weapon swapping!

---
---
## What's Done
### Robot.java
I think **autonomous** and **teleop** will work here automatically. CTRE's generation software really hooked us up I
think.

### In LockOnShootAndDrive.java
* Figure out how to get the swerve drive's current x and y velocities.
    * <mark>The answer for this is in
        [this chat](https://grok.com/share/c2hhcmQtMg_bed28fc3-0692-43fb-815a-ccc28d2ea236)</mark>

## What Needs to Be Done
* Implement `LockOnShootAndDrive::execute`.
    * Figure out how to use `SwerveRequest.FieldCentricFacingAngle`.
* Implement `Shooter::setRPM`.
* Implement `Shooter::setLaunchAngle`.
* Implement `Shooter::manualShootBall` command.
* Implement `AimCamera::getHubRelativeLocation`.
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

## Mentor Mike's Notes to Himself
`SwerveRequest.FieldCentricFacingAngle`

`SwerveRequest.FieldCentric`

`SwerveRequest.RobotCentric`

The `Collector` should definitely run on the roborio can bus. The climber probably will too. Not sure yet though on that
one.
