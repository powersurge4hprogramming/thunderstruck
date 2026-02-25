# Thunderstruck
This is the code base for the **ThunderStruck** Robot. This file acts as our ***"work-board"*** to keep track of what to
do, to communicate remotely to each other, and allow for any of my review to happen at night outside of the shop.

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
* Implement `Collector::stop` command.
    * <mark>Do we actually need this?</mark>

## What Needs to Be Tested
* The `Shooter::setRPM`.
* Give data to the `Shooter::manualShootBall` command.
* The `Shooter::manualShootBall` command.
* The various rumble commands.
* <mark>The entire autonomous plans!!!</mark>
* The `LockOnShootAndDrive` **y** binding.
* The `AimCamera::getHubRelativeLocation` function.
* The `LockOnShootAndDrive::execute` function.
* The `Collector::run` command.
* The `Climber::upward` command.
* The `Climber::downward` command.

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
