package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class FaceWall extends Command {
        // =============================================================================================================
        // Private Data Members
        // =============================================================================================================
        private final CommandSwerveDrivetrain drivetrain;
        private final SwerveRequest.FieldCentricFacingAngle fieldFacingAngle;
        private Rotation2d towerHeading;

        // =============================================================================================================
        // Package Protected Contructor
        // =============================================================================================================
        FaceWall(final double maxSpeed, final double maxAngularRate, final CommandSwerveDrivetrain drivetrain) {
                this.towerHeading = null;
                this.drivetrain = drivetrain;
                this.fieldFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
                                // Add a 10% deadband
                                .withDeadband(maxSpeed * 0.1)
                                // Use open-loop control for drive motors
                                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

                addRequirements(this.drivetrain);
        }

        // =============================================================================================================
        // Public Methods
        // =============================================================================================================
        @Override
        public void initialize() {
                final Optional<Alliance> allianceOptional = DriverStation.getAlliance();
                final Alliance alliance = allianceOptional
                                .orElseThrow(() -> new RuntimeException("Whaaaaa... No alliance is set?"));
                this.towerHeading = (alliance == Alliance.Blue) ? Rotation2d.fromDegrees(180.0)
                                : Rotation2d.fromDegrees(0.0);

                // Face the wall.
                drivetrain.setControl(fieldFacingAngle
                                .withVelocityX(0)
                                .withVelocityY(0)
                                .withTargetDirection(towerHeading));
        }

        // -------------------------------------------------------------------------------------------------------------
        @Override
        public boolean isFinished() {
                final Rotation3d current = drivetrain.getRotation3d();
                return current.minus(new Rotation3d(towerHeading)).getAngle() == 0;
        }

        // -------------------------------------------------------------------------------------------------------------
        @Override
        public void end(boolean interrupted) {
                drivetrain.setControl(new SwerveRequest.Idle());
        }

}
