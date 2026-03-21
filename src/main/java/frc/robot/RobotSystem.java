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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.rumble.RumbleDynamicCommand;
import frc.robot.commands.rumble.RumbleIntensity;
import frc.robot.commands.rumble.RumblePulseCommand;
import frc.robot.commands.shoot.LockOnShootAndDrive;
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
        private static final double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        // 3/4 of a rotation per second max angular velocity
        private static final double MaxAngularRateScaler = 0.75;
        private static final double MaxAngularRate = RotationsPerSecond.of(MaxAngularRateScaler).in(RadiansPerSecond);

        private static final String EVENT_SHOOT = "shoot";
        private static final String EVENT_COLLECT = "collect";
        private static final String EVENT_HOPPER = "hopper";

        // =============================================================================================================
        // Driver Inputs
        // =============================================================================================================
        private final CommandXboxController driver = new CommandXboxController(USB.CONTROLLER.DRIVER);
        private final CommandXboxController operator = new CommandXboxController(USB.CONTROLLER.OPERATOR);
        private boolean isLockedOn = false;
        private double maxSpeedScalar = 1.0;

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
        private static final byte BRAKE_INDEX = 0;
        private static final byte WHEEL_POINT_INDEX = 1;
        private static final byte LOCK_ON_SHOOT_AND_DRIVE_INDEX = 2;
        private static final byte MANUAL_SHOOT_INDEX = 3;
        private static final byte COLLECTOR_RUN_INDEX = 4;
        private static final byte RESET_FIELD_ORIENTATION_INDEX = 5;
        private static final byte WEAPON_SWAP_INDEX = 6;
        private static final byte FEEDER_RUN_OUT_INDEX = 7;
        private static final byte SPEED_CHANGE_INDEX = 8;
        private static final byte HOPPER_IN_INDEX = 9;
        /**
         * {@summary}
         * The purpose of this array is for cancelling the "active" commands that are in
         * it when a profile is switched.
         */
        private final Command[] commands = {
                        /* Brake */
                        null,
                        /* Wheels Point */
                        null,
                        /* Lock on Shoot and Drive */
                        null,
                        /* ManualShoot */
                        null,
                        /* Collector.run() */
                        null,
                        /* Reset Field Orientation */
                        null,
                        /* Weapon Swap */
                        null,
                        /* Manual Feeder Out */
                        null,
                        /* Speed Changing */
                        null,
                        /* Hopper In */
                        null,
        };

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
        final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
                        // add a 10% deadband
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
                        // Use open-loop control for drive motors
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

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
                                (speeds) -> drivetrain.setControl(robotDrive
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

                // NamedCommands.registerCommand(EVENT_COLLECT, collector.run(() -> 1));
                NamedCommands.registerCommand(EVENT_SHOOT, shooter.manualShootBall(() -> 1)
                                .alongWith(new WaitCommand(1)
                                                .andThen(feeder.manualFeederRunIn())
                                                .withTimeout(4.0)));
                // NamedCommands.registerCommand(EVENT_HOPPER, collector.run(() -> 1));

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
                drivetrain.setDefaultCommand(makeNormalDriveCommand(driver));
                new Trigger(DriverStation::isDisabled).whileTrue(makeIdleCommand());
        }

        // -------------------------------------------------------------------------------------------------------------
        private void defaultBindingsProfile() {
                setDefaultBindings();

                commands[BRAKE_INDEX] = makeBrakeCommand(() -> RumbleType.kLeftRumble, driver);
                commands[RESET_FIELD_ORIENTATION_INDEX] = makeResetFieldOrientationCommand(
                                () -> RumbleType.kBothRumble, driver);
                commands[WHEEL_POINT_INDEX] = makeWheelsPointCommand(() -> RumbleType.kLeftRumble, driver);
                commands[SPEED_CHANGE_INDEX] = makeMaxSpeedChangeCommand(() -> RumbleType.kRightRumble, driver);
                driver.leftBumper().whileTrue(commands[BRAKE_INDEX]);
                driver.y().onTrue(commands[RESET_FIELD_ORIENTATION_INDEX]);
                driver.povLeft().onTrue(commands[WHEEL_POINT_INDEX]);
                driver.b().onTrue(commands[SPEED_CHANGE_INDEX]);

                // ------------
                commands[COLLECTOR_RUN_INDEX] = makeCollectorRunCommand(() -> -operator.getLeftTriggerAxis(),
                                () -> RumbleType.kLeftRumble, operator);
                commands[MANUAL_SHOOT_INDEX] = makeManualShootCommand(() -> operator.getRightTriggerAxis(),
                                () -> RumbleType.kRightRumble, operator);
                commands[LOCK_ON_SHOOT_AND_DRIVE_INDEX] = makeLockOnShootAndDriveCommand(() -> RumbleType.kBothRumble,
                                operator);
                commands[WEAPON_SWAP_INDEX] = makeWeaponSwapCommand(() -> RumbleType.kBothRumble, operator);
                commands[HOPPER_IN_INDEX] = makeManualFeederInCommand(() -> RumbleType.kLeftRumble, operator);
                commands[FEEDER_RUN_OUT_INDEX] = makeManualFeederOutCommand(() -> RumbleType.kLeftRumble, operator);
                operator.leftTrigger().whileTrue(commands[COLLECTOR_RUN_INDEX]);
                operator.rightTrigger()
                                .and(() -> isLockedOn == false)
                                .whileTrue(commands[MANUAL_SHOOT_INDEX]);
                operator.x().toggleOnTrue(commands[WEAPON_SWAP_INDEX]);
                operator.a().whileTrue(commands[HOPPER_IN_INDEX]);
                operator.b()
                                .and(() -> isLockedOn == false)
                                .whileTrue(commands[FEEDER_RUN_OUT_INDEX]);

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
                                        .withRotationalRate(-controller.getRightX() * MaxAngularRate);
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
        private Command makeBrakeCommand(final Supplier<RumbleType> side, final CommandXboxController controller) {
                return new ParallelCommandGroup(drivetrain.applyRequest(() -> brake),
                                RumblePulseCommand.createLongSinglePulse(controller, RumbleIntensity.MEDIUM_LIGHT,
                                                side).handleInterrupt(() -> controller.setRumble(side.get(), 0)));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeWheelsPointCommand(final Supplier<RumbleType> side,
                        final CommandXboxController controller) {
                /*
                 * NOTE:
                 * All this does it point the wheels to whatever direction it is controlled to
                 * point towards. Not sure of its usefulness.
                 * 
                 * Point the wheels to the zero position.
                 */
                final Command zeroWheels = drivetrain.applyRequest(
                                () -> point.withModuleDirection(
                                                new Rotation2d(0, 0)));
                return new ParallelCommandGroup(zeroWheels,
                                RumblePulseCommand.createShortSinglePulse(controller, RumbleIntensity.VERY_LIGHT,
                                                side).handleInterrupt(() -> controller.setRumble(side.get(), 0)));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeLockOnShootAndDriveCommand(final Supplier<RumbleType> side,
                        final CommandXboxController controller) {
                return new LockOnShootAndDrive(
                                shooter,
                                drivetrain,
                                feeder,
                                aimCamera,
                                () -> -driver.getLeftX() * MaxSpeed * 0.10,
                                () -> -driver.getLeftY() * MaxSpeed * 0.20,
                                MaxSpeed)
                                .handleInterrupt(() -> {
                                        System.out.println("I am wondering if this executes on cancel()?");
                                        getCommandScheduler().schedule(RumblePulseCommand
                                                        .createShortDoublePulse(controller,
                                                                        RumbleIntensity.SUPER_HEAVY,
                                                                        side)
                                                        .handleInterrupt(() -> controller
                                                                        .setRumble(side.get(), 0)));
                                        isLockedOn = false;
                                });
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
        private Command makeWeaponSwapCommand(final Supplier<RumbleType> side, final CommandXboxController controller) {
                final Command weaponSwap = new InstantCommand(() -> {
                        System.out.println("Swapping.");
                        if (!isLockedOn) {
                                System.out.println("Currently manual, swapping to lock on.");
                                if (commands[MANUAL_SHOOT_INDEX].isScheduled()) {
                                        getCommandScheduler().cancel(commands[MANUAL_SHOOT_INDEX]);
                                }
                                System.out.println("Swapping to lock on.");
                                getCommandScheduler().schedule(commands[LOCK_ON_SHOOT_AND_DRIVE_INDEX]);
                                isLockedOn = true;
                        } else {
                                System.out.println("Currently locked on, swapping to manual.");
                                if (commands[LOCK_ON_SHOOT_AND_DRIVE_INDEX].isScheduled()) {
                                        getCommandScheduler().cancel(
                                                        commands[LOCK_ON_SHOOT_AND_DRIVE_INDEX]);
                                }
                                System.out.println("Swapping to manual.");
                                isLockedOn = false;
                        }
                });

                return new ParallelCommandGroup(weaponSwap,
                                RumblePulseCommand.createShortSinglePulse(controller, RumbleIntensity.SUPER_HEAVY,
                                                side)
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
        private Command makeMaxSpeedChangeCommand(final Supplier<RumbleType> side,
                        final CommandXboxController controller) {
                return new ParallelCommandGroup(new InstantCommand(() -> {
                        if (maxSpeedScalar == 1) {
                                maxSpeedScalar = 0.33;
                        } else if (maxSpeedScalar == 0.66) {
                                maxSpeedScalar = 1;
                        } else if (maxSpeedScalar == 0.33) {
                                maxSpeedScalar = 0.66;
                        }
                }), RumblePulseCommand.createShortSinglePulse(controller, RumbleIntensity.MEDIUM_HEAVY, side));
        }

}