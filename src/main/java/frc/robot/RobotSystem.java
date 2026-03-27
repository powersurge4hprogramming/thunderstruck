// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.rumble.RumbleDynamicCommand;
import frc.robot.commands.rumble.RumbleIntensity;
import frc.robot.commands.rumble.RumblePulseCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.vision.AimCamera;
import frc.robot.subsystems.Collector;

public class RobotSystem {
        // =============================================================================================================
        // Constants
        // =============================================================================================================
        // kSpeedAt12Volts desired top speed
        private static final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private static final double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);

        private static final String EVENT_SHOOT = "shoot";
        private static final String EVENT_COLLECT = "collect";
        private static final String EVENT_HOPPER = "hopper";

        // =============================================================================================================
        // Driver Inputs
        // =============================================================================================================
        private final CommandXboxController driver = new CommandXboxController(USB.CONTROLLER.DRIVER);
        private final CommandXboxController operator = new CommandXboxController(USB.CONTROLLER.OPERATOR);
        private double maxSpeedScalar = 0.5;
        private double maxRotSpeedScalar = 0.75;

        // =============================================================================================================
        // Systems
        // =============================================================================================================
        private final AimCamera aimCamera = new AimCamera();

        // =============================================================================================================
        // Sub-Systems
        // =============================================================================================================
        private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final Shooter shooter = new Shooter();
        private final Feeder feeder = new Feeder();
        private final Collector collector = new Collector();

        // =============================================================================================================
        // Commands
        // =============================================================================================================
        private static final byte STASIS_INDEX = 0;
        private static final byte MANUAL_SHOOT_INDEX = 1;
        private static final byte COLLECTOR_RUN_INDEX = 2;
        private static final byte RESET_FIELD_ORIENTATION_INDEX = 3;
        private static final byte FEEDER_RUN_OUT_INDEX = 4;
        private static final byte FEEDER_IN_INDEX = 5;
        private static final byte DRIVE_SPEED_UP_INDEX = 6;
        private static final byte DRIVE_SPEED_DOWN_INDEX = 7;
        private static final byte DRIVE_SPEED_MAX_INDEX = 8;
        private static final byte DRIVE_SPEED_DEFAULT_INDEX = 9;
        private static final byte DRIVE_ANGLE_SPEED_UP_INDEX = 10;
        private static final byte DRIVE_ANGLE_SPEED_DOWN_INDEX = 11;
        private static final byte DRIVE_ANGLE_SPEED_MAX_INDEX = 12;
        private static final byte DRIVE_ANGLE_SPEED_DEFAULT_INDEX = 13;
        private final Command[] commands = new Command[14];

        // =============================================================================================================
        // PathPlanner
        // =============================================================================================================
        private final SendableChooser<Command> autoChooser;

        // =============================================================================================================
        // Swerve Drive Configurations
        // =============================================================================================================
        final SwerveRequest.FieldCentric fieldDrive = new SwerveRequest.FieldCentric()
                        // Add a 10% deadband
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
                        // Use open-loop control for drive motors
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        final SwerveRequest.RobotCentric robotDriveAuto = new SwerveRequest.RobotCentric()
                        // add a 10% deadband
                        .withDeadband(0).withRotationalDeadband(0)
                        // Use open-loop control for drive motors
                        .withDriveRequestType(DriveRequestType.Velocity);
        final SwerveRequest.SwerveDriveBrake stasis = new SwerveRequest.SwerveDriveBrake();

        // =============================================================================================================
        // Logging
        // =============================================================================================================
        private final Telemetry logger = new Telemetry(MaxSpeed);

        // =============================================================================================================
        // The Constructor
        // =============================================================================================================
        public RobotSystem() {
                drivetrain.setAimCamera(aimCamera);

                defaultBindingsProfile();

                // Robot config: Pull from PathPlanner GUI settings (tune mass ~40-50kg for your
                // bot, MOI from CAD, module drive ratios from SDS/Krakens)
                RobotConfig robotConfig = null;
                try {
                        // Auto-loads from deploy/settings.json; run
                        // PathPlanner app connected to bot first
                        robotConfig = RobotConfig.fromGUISettings();
                } catch (IOException | ParseException e) {
                        e.printStackTrace();
                        System.exit(0);
                }
                AutoBuilder.configure(
                                drivetrain::getPose, // Fused pose supplier (vision-corrected)
                                drivetrain::resetPose, // Pose resetter
                                drivetrain::getChassisSpeeds, // ChassisSpeeds supplier (from CTRE state)
                                (speeds) -> drivetrain.setControl(robotDriveAuto
                                                .withVelocityX(speeds.vxMetersPerSecond)
                                                .withVelocityY(speeds.vyMetersPerSecond)
                                                .withRotationalRate(speeds.omegaRadiansPerSecond)),
                                new PPHolonomicDriveController(
                                                // Translation PID (P=1.0 start; tune higher for aggression)
                                                new PIDConstants(1.0, 0.0, 0.0),
                                                // Rotation PID (P=2.0 start; tune for turns)
                                                new PIDConstants(2.0, 0.0, 0.0)),
                                robotConfig, // Robot config for dynamics
                                // Flip paths for red
                                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                                drivetrain // Subsystem ref
                );

                NamedCommands.registerCommand(EVENT_COLLECT, collector.run(() -> -1).withTimeout(4));
                NamedCommands.registerCommand(EVENT_SHOOT, shooter.manualShootBall(() -> 1)
                                .alongWith(new WaitCommand(1)
                                                .andThen(feeder.manualFeederRunIn())
                                                .withTimeout(4.0))
                                .withTimeout(7));
                /*
                 * NamedCommands.registerCommand(EVENT_HOPPER,
                 * new RunCommand(() -> collector.setCollector()).withTimeout(2)
                 * .finallyDo(() -> collector.stopCollector())
                 * .andThen(collector.run(() -> -1))
                 * .withTimeout(1.5));
                 */
                NamedCommands.registerCommand(EVENT_HOPPER, collector.run(() -> -1).withTimeout(1.5));

                // Setup the auto UI in Shuffleboard.
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        // =============================================================================================================
        // Public Methods
        // =============================================================================================================
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        // -------------------------------------------------------------------------------------------------------------
        public CommandScheduler getCommandScheduler() {
                return CommandScheduler.getInstance();
        }

        // -------------------------------------------------------------------------------------------------------------
        public void updatePhotonCameraFrames() {
                aimCamera.updateFrames();
        }

        // =============================================================================================================
        // Private Methods
        // =============================================================================================================
        private void setDefaultBindings() {
                /*
                 * IMPORTANT!
                 * 
                 * This setDefaultCommand method's behavior is not tied to the event loop and
                 * can therefore be set outside the profiles. Plus, it never changes.
                 */
                new Trigger(DriverStation::isDisabled).whileTrue(makeIdleCommand());
        }

        // -------------------------------------------------------------------------------------------------------------
        private void defaultBindingsProfile() {
                setDefaultBindings();

                commands[RESET_FIELD_ORIENTATION_INDEX] = makeResetFieldOrientationCommand(
                                () -> RumbleType.kRightRumble, driver);
                commands[STASIS_INDEX] = makeStasisCommand(() -> RumbleType.kLeftRumble, driver);

                drivetrain.setDefaultCommand(makeNormalDriveCommand(driver));
                driver.leftBumper().whileTrue(commands[STASIS_INDEX]);
                driver.rightBumper().onTrue(commands[RESET_FIELD_ORIENTATION_INDEX]);

                // ------------
                commands[COLLECTOR_RUN_INDEX] = makeCollectorRunCommand(() -> -operator.getLeftTriggerAxis(),
                                () -> RumbleType.kLeftRumble, operator);
                commands[MANUAL_SHOOT_INDEX] = makeManualShootCommand(() -> operator.getRightTriggerAxis(),
                                () -> RumbleType.kRightRumble, operator);
                commands[FEEDER_IN_INDEX] = makeManualFeederInCommand(() -> RumbleType.kLeftRumble, operator);
                commands[FEEDER_RUN_OUT_INDEX] = makeManualFeederOutCommand(() -> RumbleType.kLeftRumble, operator);
                commands[DRIVE_SPEED_UP_INDEX] = makeMaxDriveSpeedGoUpCommand(() -> RumbleType.kRightRumble, operator);
                commands[DRIVE_SPEED_DOWN_INDEX] = makeMaxDriveSpeedGoDownCommand(() -> RumbleType.kLeftRumble,
                                operator);
                commands[DRIVE_SPEED_DEFAULT_INDEX] = makeMaxDriveSpeedDefaultCommand(() -> RumbleType.kLeftRumble,
                                operator);
                commands[DRIVE_SPEED_MAX_INDEX] = makeMaxDriveSpeedFullCommand(() -> RumbleType.kRightRumble, operator);
                commands[DRIVE_ANGLE_SPEED_UP_INDEX] = makeMaxAngleDriveSpeedGoUpCommand(() -> RumbleType.kRightRumble,
                                operator);
                commands[DRIVE_ANGLE_SPEED_DOWN_INDEX] = makeMaxAngleDriveSpeedGoDownCommand(
                                () -> RumbleType.kLeftRumble,
                                operator);
                commands[DRIVE_ANGLE_SPEED_DEFAULT_INDEX] = makeMaxAngleDriveSpeedDefaultCommand(
                                () -> RumbleType.kLeftRumble,
                                operator);
                commands[DRIVE_ANGLE_SPEED_MAX_INDEX] = makeMaxAngleDriveSpeedFullCommand(() -> RumbleType.kRightRumble,
                                operator);
                operator.rightTrigger().whileTrue(commands[MANUAL_SHOOT_INDEX]);
                operator.leftTrigger().whileTrue(commands[COLLECTOR_RUN_INDEX]);
                operator.rightBumper().whileTrue(commands[FEEDER_IN_INDEX]);
                operator.leftBumper().whileTrue(commands[FEEDER_RUN_OUT_INDEX]);
                operator.povUp().onTrue(commands[DRIVE_SPEED_UP_INDEX]);
                operator.povDown().onTrue(commands[DRIVE_SPEED_DOWN_INDEX]);
                operator.povRight().onTrue(commands[DRIVE_SPEED_MAX_INDEX]);
                operator.povLeft().onTrue(commands[DRIVE_SPEED_DEFAULT_INDEX]);
                operator.y().onTrue(commands[DRIVE_ANGLE_SPEED_UP_INDEX]);
                operator.a().onTrue(commands[DRIVE_ANGLE_SPEED_DOWN_INDEX]);
                operator.b().onTrue(commands[DRIVE_ANGLE_SPEED_MAX_INDEX]);
                operator.x().onTrue(commands[DRIVE_ANGLE_SPEED_DEFAULT_INDEX]);
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeNormalDriveCommand(final CommandXboxController controller) {
                /*
                 * Note that X is defined as forward according to WPILib convention, and Y is
                 * defined as to the left according to WPILib convention.
                 * 
                 * This command will always run on the drivetrain until another command takes
                 * control of it.
                 */

                // Drivetrain will execute this command periodically
                return drivetrain.applyRequest(() -> {
                        // Drive forward with negative Y (forward)
                        return fieldDrive.withVelocityX(-controller.getLeftY() * MaxSpeed * maxSpeedScalar)
                                        // Drive left with negative X (left)
                                        .withVelocityY(-controller.getLeftX() * MaxSpeed * maxSpeedScalar)
                                        // Drive counterclockwise with negative X (left)
                                        .withRotationalRate(
                                                        -controller.getRightX() * MaxAngularRate * maxRotSpeedScalar);
                });
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
        private Command makeStasisCommand(final Supplier<RumbleType> side, final CommandXboxController controller) {
                return new ParallelCommandGroup(drivetrain.applyRequest(() -> stasis),
                                RumblePulseCommand.createLongSinglePulse(controller, RumbleIntensity.MEDIUM_LIGHT,
                                                side).handleInterrupt(() -> controller.setRumble(side.get(), 0)));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeManualShootCommand(final DoubleSupplier ballVelocityScalar,
                        final Supplier<RumbleType> side, final CommandXboxController controller) {
                return new ParallelCommandGroup(shooter.manualShootBall(ballVelocityScalar),
                                new RumbleDynamicCommand(controller, ballVelocityScalar, side)
                                                .handleInterrupt(() -> controller.setRumble(side.get(), 0)));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeManualFeederInCommand(final Supplier<RumbleType> side,
                        final CommandXboxController controller) {
                return new ParallelCommandGroup(
                                feeder.manualFeederRunIn(),
                                new RumbleDynamicCommand(controller, () -> RumbleIntensity.MEDIUM, side)
                                                .handleInterrupt(() -> controller.setRumble(side.get(), 0)));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeManualFeederOutCommand(final Supplier<RumbleType> side,
                        final CommandXboxController controller) {
                return new ParallelCommandGroup(
                                feeder.manualFeederRunOut(),
                                new RumbleDynamicCommand(controller, () -> RumbleIntensity.MEDIUM, side)
                                                .handleInterrupt(() -> controller.setRumble(side.get(), 0)));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeCollectorRunCommand(final DoubleSupplier collectorScalar, final Supplier<RumbleType> side,
                        final CommandXboxController controller) {
                return new ParallelCommandGroup(collector.run(collectorScalar),
                                new RumbleDynamicCommand(controller, collectorScalar, side)
                                                .handleInterrupt(() -> controller.setRumble(side.get(), 0)));

        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeResetFieldOrientationCommand(final Supplier<RumbleType> side,
                        final CommandXboxController controller) {
                // Reset the field-centric heading on left bumper press.
                return new ParallelCommandGroup(drivetrain.runOnce(drivetrain::seedFieldCentric),
                                RumblePulseCommand.createLongDoublePulse(controller, RumbleIntensity.MEDIUM_HEAVY,
                                                side).handleInterrupt(() -> controller.setRumble(side.get(), 0)));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeMaxDriveSpeedGoDownCommand(final Supplier<RumbleType> side,
                        final CommandXboxController controller) {
                return new ParallelCommandGroup(new InstantCommand(() -> {
                        if (maxSpeedScalar == 0.2) {
                                return;
                        }
                        maxSpeedScalar = maxSpeedScalar - 0.1;
                }), RumblePulseCommand.createShortSinglePulse(controller, RumbleIntensity.MEDIUM_HEAVY, side));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeMaxDriveSpeedGoUpCommand(final Supplier<RumbleType> side,
                        final CommandXboxController controller) {
                return new ParallelCommandGroup(new InstantCommand(() -> {
                        if (maxSpeedScalar == 1) {
                                return;
                        }
                        maxSpeedScalar = maxSpeedScalar + 0.1;
                }), RumblePulseCommand.createShortSinglePulse(controller, RumbleIntensity.MEDIUM_HEAVY, side));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeMaxDriveSpeedDefaultCommand(final Supplier<RumbleType> side,
                        final CommandXboxController controller) {
                return new ParallelCommandGroup(new InstantCommand(() -> {
                        maxSpeedScalar = 0.5;
                }), RumblePulseCommand.createShortSinglePulse(controller, RumbleIntensity.MEDIUM_HEAVY, side));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeMaxDriveSpeedFullCommand(final Supplier<RumbleType> side,
                        final CommandXboxController controller) {
                return new ParallelCommandGroup(new InstantCommand(() -> {
                        maxSpeedScalar = 1;
                }), RumblePulseCommand.createShortSinglePulse(controller, RumbleIntensity.MEDIUM_HEAVY, side));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeMaxAngleDriveSpeedGoDownCommand(final Supplier<RumbleType> side,
                        final CommandXboxController controller) {
                return new ParallelCommandGroup(new InstantCommand(() -> {
                        if (maxRotSpeedScalar == 0.2) {
                                return;
                        }
                        maxRotSpeedScalar = maxRotSpeedScalar - 0.1;
                }), RumblePulseCommand.createShortSinglePulse(controller, RumbleIntensity.MEDIUM_HEAVY, side));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeMaxAngleDriveSpeedGoUpCommand(final Supplier<RumbleType> side,
                        final CommandXboxController controller) {
                return new ParallelCommandGroup(new InstantCommand(() -> {
                        if (maxRotSpeedScalar == 1) {
                                return;
                        }
                        maxRotSpeedScalar = maxRotSpeedScalar + 0.1;
                }), RumblePulseCommand.createShortSinglePulse(controller, RumbleIntensity.MEDIUM_HEAVY, side));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeMaxAngleDriveSpeedDefaultCommand(final Supplier<RumbleType> side,
                        final CommandXboxController controller) {
                return new ParallelCommandGroup(new InstantCommand(() -> {
                        maxRotSpeedScalar = 0.75;
                }), RumblePulseCommand.createShortSinglePulse(controller, RumbleIntensity.MEDIUM_HEAVY, side));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeMaxAngleDriveSpeedFullCommand(final Supplier<RumbleType> side,
                        final CommandXboxController controller) {
                return new ParallelCommandGroup(new InstantCommand(() -> {
                        maxRotSpeedScalar = 1;
                }), RumblePulseCommand.createShortSinglePulse(controller, RumbleIntensity.MEDIUM_HEAVY, side));
        }

}