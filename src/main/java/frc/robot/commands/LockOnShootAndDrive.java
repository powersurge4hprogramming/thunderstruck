package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.AimCamera;

public class LockOnShootAndDrive extends Command {
    private final Shooter shooter;
    private final CommandSwerveDrivetrain drive;
    private final AimCamera aimCamera;
    // Allow driver to still move X/Y
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    public LockOnShootAndDrive(Shooter shooter, CommandSwerveDrivetrain drive, AimCamera aimCamera,
            DoubleSupplier x, DoubleSupplier y) {
        this.shooter = shooter;
        this.drive = drive;
        this.aimCamera = aimCamera;
        this.xSupplier = x;
        this.ySupplier = y;

        // REQUIRE BOTH: This stops any other drive or shooter commands
        addRequirements(shooter, drive);
    }

    @Override
    public void execute() {
        throw new RuntimeException("Not implemented yet.");
        // 1. One physics calculation to rule them all
        // 2. Command the Shooter
        // 3. Command the Drivebase
        // We use the driver's X/Y but OVERRIDE the rotation with our 3D solution
    }
}