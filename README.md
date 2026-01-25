# Thunderstruck

## What Needs to Be Worked On Today
Today we are going to be *"hooking up"* (still trying to think of a better name for this) the
[LockOnShootAndDrive](src/main/java/frc/robot/commands/LockOnShootAndDrive.java) command to the controller.

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
