# Thunderstruck
This is the code base for the **ThunderStruck** Robot. This file acts as our ***"work-board"*** to keep track of what to
do, to communicate remotely to each other, and allow for any of my review to happen at night outside of the shop.

## What's Done
* Finish up the optimizations for the autos.
* Mark all of the events in the autos.
* The `Shooter::manualShootBall` command.
* The `Shooter::setRPM`.
* The `Collector::run` command.

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
* The `LockOnClimb` command.
* The `Hopper::unclasp` command.
* the `eventsAuto` map.
* Give data to the `Shooter::manualShootBall` command.
* The various rumble commands.
* <mark>The entire autonomous plans!!!</mark>
* The ***weapon swap*** binding.
* The `AimCamera::getHubRelativeLocation` function.
* The `LockOnShootAndDrive::execute` function.
* The `Climber::upward` command.
* The `Climber::downward` command.

## Mentor Mike's Notes to Himself
