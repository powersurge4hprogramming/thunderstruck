// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.LockOnShootAndDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.AimCamera;
import frc.robot.subsystems.Collector;

public class RobotSystem {
        // =============================================================================================================
        // Constants
        // =============================================================================================================
        // kSpeedAt12Volts desired top speed
        private static double MaxSpeedScaler = 0.25;
        private static double MaxSpeed = MaxSpeedScaler * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        // 3/4 of a rotation per second max angular velocity
        private static double MaxAngularRateScaler = 0.75;
        private static double MaxAngularRate = RotationsPerSecond.of(MaxAngularRateScaler).in(RadiansPerSecond);

        // =============================================================================================================
        // Sub-Systems
        // =============================================================================================================
        private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final Shooter shooter = new Shooter();
        private final AimCamera aimCamera = new AimCamera();
        private final Collector Collector = new Collector();

        // =============================================================================================================
        // Commands
        // =============================================================================================================
        private static final byte NORMAL_DRIVE_INDEX = 0;
        private static final byte IDLE_INDEX = 1;
        private static final byte BRAKE_INDEX = 2;
        private static final byte WHEEL_POINT_INDEX = 3;
        private static final byte LOCK_ON_SHOOT_AND_DRIVE_INDEX = 4;
        private static final byte MANUAL_SHOOT_INDEX = 5;
        private static final byte COLLECTOR_RUN_INDEX = 6;
        private static final byte RESET_FIELD_ORIENTATION_INDEX = 7;
        private static final byte SYSID_DYNAMIC_FORWARD_INDEX = 8;
        private static final byte SYSID_DYNAMIC_REVERSE_INDEX = 9;
        private static final byte SYSID_QUASISTATIC_FORWARD_INDEX = 10;
        private static final byte SYSID_QUASISTATIC_REVERSE_INDEX = 11;
        private static final byte WEAPON_SWAP_INDEX = 12;
        private final Command[] commands = {
                        makeNormalDriveCommand(),
                        makeIdleCommand(),
                        makeBrakeCommand(),
                        makeWheelsPointCommand(),
                        makeLockOnShootAndDriveCommand(),
                        makeManualShootCommand(),
                        makeCollectorRunCommand(),
                        makeResetFieldOrientationCommand(),
                        makeSysIdDynamicForwardCommand(),
                        makeSysIdDynamicReverseCommand(),
                        makeSysIdQuasistaticForwardCommand(),
                        makeSysIdQuasistaticReverseCommand(),
                        makeWeaponSwapCommand(),
        };

        // =============================================================================================================
        // Swerve Drive Configurations
        // =============================================================================================================
        final SwerveRequest.FieldCentric fieldDrive = new SwerveRequest.FieldCentric()
                        // Add a 10% deadband
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
                        // Use open-loop control for drive motors
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

        final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        // =============================================================================================================
        // Modules
        // =============================================================================================================
        private final PowerDistribution powerDistribution = new PowerDistribution(CANBus.ID.POWER_DISTRIBUTION,
                        ModuleType.kRev);

        // =============================================================================================================
        // Logging
        // =============================================================================================================
        private final Telemetry logger = new Telemetry(MaxSpeed);

        // =============================================================================================================
        // Driver Inputs
        // =============================================================================================================
        private final CommandXboxController controller = new CommandXboxController(USB.CONTROLLER.DRIVER);
        private boolean checkAimbotStatus = false;

        // =============================================================================================================
        // The Constructor
        // =============================================================================================================
        public RobotSystem() {
                defaultBindingsProfile();
                drivetrain.registerTelemetry(logger::telemeterize);
        }

        // =============================================================================================================
        // Public Methods
        // =============================================================================================================
        public Command getAutonomousCommand() {
                // Simple drive forward auton
                final var idle = new SwerveRequest.Idle();
                return Commands.sequence(
                                // Reset our field centric heading to match the robot
                                // facing away from our alliance station wall (0 deg).
                                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                                // Then slowly drive forward (away from us) for 5 seconds.
                                drivetrain.applyRequest(() -> fieldDrive.withVelocityX(0.5)
                                                .withVelocityY(0)
                                                .withRotationalRate(0))
                                                .withTimeout(5.0),
                                // Finally idle for the rest of auton
                                drivetrain.applyRequest(() -> idle));
        }

        // -------------------------------------------------------------------------------------------------------------
        public CommandScheduler getCommandScheduler() {
                return CommandScheduler.getInstance();
        }

        // =============================================================================================================
        // Private Methods
        // =============================================================================================================
        private void defaultBindingsProfile() {
                setDefaultBindings();

                controller.a().whileTrue(commands[BRAKE_INDEX]);
                controller.b().whileTrue(commands[WHEEL_POINT_INDEX]);
                controller.y().onTrue(commands[WEAPON_SWAP_INDEX]);
                controller.rightTrigger().onTrue(commands[COLLECTOR_RUN_INDEX]);
                controller.leftBumper().onTrue(commands[RESET_FIELD_ORIENTATION_INDEX]);
                /*
                 * Run SysId routines when holding back/start and X/Y. Note that each routine
                 * should be run exactly once in a single log.
                 */
                controller.back().and(controller.y()).whileTrue(commands[SYSID_DYNAMIC_FORWARD_INDEX]);
                controller.back().and(controller.x()).whileTrue(commands[SYSID_DYNAMIC_REVERSE_INDEX]);
                controller.start().and(controller.y()).whileTrue(commands[SYSID_QUASISTATIC_FORWARD_INDEX]);
                controller.start().and(controller.x()).whileTrue(commands[SYSID_QUASISTATIC_REVERSE_INDEX]);
        }

        // -------------------------------------------------------------------------------------------------------------
        private void setDefaultBindings() {
                drivetrain.setDefaultCommand(commands[NORMAL_DRIVE_INDEX]);
                RobotModeTriggers.disabled().whileTrue(commands[IDLE_INDEX]);
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeNormalDriveCommand() {
                /*
                 * Note that X is defined as forward according to WPILib convention, and Y is
                 * defined as to the left according to WPILib convention.
                 * 
                 * This command will always run on the drivetrain until another command takes
                 * control of it.
                 */
                // Drivetrain will execute this command periodically
                return drivetrain.applyRequest(() ->
                // Drive forward with negative Y (forward)
                fieldDrive.withVelocityX(-controller.getLeftY() * MaxSpeed)
                                // Drive left with negative X (left)
                                .withVelocityY(-controller.getLeftX() * MaxSpeed)
                                // Drive counterclockwise with negative X (left)
                                .withRotationalRate(-controller.getRightX() * MaxAngularRate));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeIdleCommand() {
                /*
                 * Idle while the robot is disabled. This ensures the configured neutral mode is
                 * applied to the drive motors while disabled.
                 */
                final var idle = new SwerveRequest.Idle();
                return drivetrain.applyRequest(() -> idle).ignoringDisable(true);
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeBrakeCommand() {
                return drivetrain.applyRequest(() -> brake);
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeWheelsPointCommand() {
                /*
                 * NOTE:
                 * All this does it point the wheels to whatever direction it is controlled to
                 * point towards. Not sure of its usefulness. Can either of you think of one?
                 */
                return drivetrain.applyRequest(
                                () -> point.withModuleDirection(
                                                new Rotation2d(-controller.getLeftY(), -controller.getLeftX())));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeLockOnShootAndDriveCommand() {
                return new LockOnShootAndDrive(
                                shooter,
                                drivetrain,
                                aimCamera,
                                () -> -controller.getLeftX() * MaxSpeed,
                                () -> -controller.getLeftY() * MaxSpeed,
                                () -> powerDistribution.getVoltage(),
                                MaxSpeed)
                                .handleInterrupt(() -> {
                                        checkAimbotStatus = true;
                                        getCommandScheduler().schedule(commands[MANUAL_SHOOT_INDEX]);
                                });
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeManualShootCommand() {
                return shooter.manualShootBall(() -> 0.5);
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeWeaponSwapCommand() {
                return new InstantCommand(() -> {
                        if (checkAimbotStatus == true) {
                                if (commands[MANUAL_SHOOT_INDEX].isScheduled()) {
                                        getCommandScheduler().cancel(commands[MANUAL_SHOOT_INDEX]);
                                }
                                getCommandScheduler().schedule(commands[LOCK_ON_SHOOT_AND_DRIVE_INDEX]);
                                checkAimbotStatus = false;
                        } else {
                                if (commands[LOCK_ON_SHOOT_AND_DRIVE_INDEX].isScheduled()) {
                                        getCommandScheduler().cancel(
                                                        commands[LOCK_ON_SHOOT_AND_DRIVE_INDEX]);
                                }
                                getCommandScheduler().schedule(commands[MANUAL_SHOOT_INDEX]);
                                checkAimbotStatus = true;
                        }
                });
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeCollectorRunCommand() {
                return Collector.run(() -> controller.getRightTriggerAxis());
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeResetFieldOrientationCommand() {
                // Reset the field-centric heading on left bumper press.
                return drivetrain.runOnce(drivetrain::seedFieldCentric);
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeSysIdDynamicForwardCommand() {
                return drivetrain.sysIdDynamic(Direction.kForward);
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeSysIdDynamicReverseCommand() {
                return drivetrain.sysIdDynamic(Direction.kReverse);
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeSysIdQuasistaticForwardCommand() {
                return drivetrain.sysIdQuasistatic(Direction.kForward);
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeSysIdQuasistaticReverseCommand() {
                return drivetrain.sysIdQuasistatic(Direction.kReverse);
        }
}