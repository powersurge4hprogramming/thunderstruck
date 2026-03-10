// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.climb.LockOnClimb;
import frc.robot.commands.rumble.RumbleDynamicCommand;
import frc.robot.commands.rumble.RumbleIntensity;
import frc.robot.commands.rumble.RumblePulseCommand;
import frc.robot.commands.shoot.LockOnShootAndDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Feeder;
import frc.robot.vision.AimCamera;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Climber;

public class RobotSystem {
        // =============================================================================================================
        // Constants
        // =============================================================================================================
        // kSpeedAt12Volts desired top speed
        private static final double MaxSpeedScaler = 0.25;
        private static final double MaxSpeed = MaxSpeedScaler * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        // 3/4 of a rotation per second max angular velocity
        private static final double MaxAngularRateScaler = 0.75;
        private static final double MaxAngularRate = RotationsPerSecond.of(MaxAngularRateScaler).in(RadiansPerSecond);

        private static final String EVENT_SHOOT = "shoot";
        private static final String EVENT_COLLECT = "collect";
        private static final String EVENT_CLIMB = "climb";

        // =============================================================================================================
        // Driver Inputs
        // =============================================================================================================
        private final CommandXboxController controller = new CommandXboxController(USB.CONTROLLER.DRIVER);
        private boolean checkAimbotStatus = false;

        // =============================================================================================================
        // Systems
        // =============================================================================================================
        private final AimCamera aimCamera = new AimCamera();

        // =============================================================================================================
        // Sub-Systems
        // =============================================================================================================
        private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final Shooter shooter = new Shooter();
        private final Feeder loader = new Feeder();
        private final Collector collector = new Collector();
        private final Climber climber = new Climber();
        private final Hopper hopper = new Hopper();

        // =============================================================================================================
        // Modules
        // =============================================================================================================
        private final PowerDistribution powerDistribution = new PowerDistribution(CANBus.ID.POWER_DISTRIBUTION,
                        ModuleType.kRev);

        // =============================================================================================================
        // Commands
        // =============================================================================================================
        private static final byte BRAKE_RUMBLE_INDEX = 0;
        private static final byte WHEEL_POINT_RUMBLE_INDEX = 1;
        private static final byte MANUAL_SHOOT_RUMBLE_INDEX = 2;
        private static final byte COLLECTOR_RUN_RUMBLE_INDEX = 3;
        private static final byte RESET_FIELD_ORIENTATION_RUMBLE_INDEX = 4;
        private static final byte WEAPON_SWAP_RUMBLE_INDEX = 5;
        private static final byte CLIMBER_UP_RUMBLE_INDEX = 6;
        private static final byte CLIMBER_DOWN_RUMBLE_INDEX = 7;
        private static final byte HOPPER_RUN_RUMBLE_INDEX = 8;
        private static final byte FEEDER_RUN_RUMBLE_INDEX = 9;
        private static final byte PROFILE_BACK_RUMBLE_INDEX = 10;
        private static final byte PROFILE_FORWARD_RUMBLE_INDEX = 11;
        private final List<Supplier<RumbleType>> rumbles = new ArrayList<>(PROFILE_FORWARD_RUMBLE_INDEX + 1);

        // -------------------------------------------------------------------------------------------------------------
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
        private static final byte PROFILE_INCREASE = 13;
        private static final byte PROFILE_DECREASE = 14;
        private static final byte CLIMBER_UP_INDEX = 15;
        private static final byte CLIMBER_DOWN_INDEX = 16;
        private static final byte HOPPER_RUN_INDEX = 17;
        private static final byte FEEDER_RUN_INDEX = 18;
        private final Command[] commands = {
                        makeNormalDriveCommand(),
                        makeIdleCommand(),
                        makeBrakeCommand(rumbles.get(BRAKE_RUMBLE_INDEX)),
                        makeWheelsPointCommand(rumbles.get(WHEEL_POINT_RUMBLE_INDEX)),
                        makeLockOnShootAndDriveCommand(),
                        /* ManualShoot */
                        null, // A command with an input driver set by the profile.
                        /* Collector.run() */
                        null, // A command with an input driver set by the profile.
                        makeResetFieldOrientationCommand(rumbles.get(RESET_FIELD_ORIENTATION_RUMBLE_INDEX)),
                        makeSysIdDynamicForwardCommand(),
                        makeSysIdDynamicReverseCommand(),
                        makeSysIdQuasistaticForwardCommand(),
                        makeSysIdQuasistaticReverseCommand(),
                        makeWeaponSwapCommand(rumbles.get(WEAPON_SWAP_RUMBLE_INDEX)),
                        makeProfileIncreaseCommand(rumbles.get(PROFILE_FORWARD_RUMBLE_INDEX)),
                        makeProfileDecreaseCommand(rumbles.get(PROFILE_BACK_RUMBLE_INDEX)),
                        makeClimberUpCommand(rumbles.get(CLIMBER_UP_RUMBLE_INDEX)),
                        makeClimberDownCommand(rumbles.get(CLIMBER_DOWN_RUMBLE_INDEX)),
                        makeHopperRunCommand(rumbles.get(HOPPER_RUN_RUMBLE_INDEX)),
                        makeManualFeederCommand(rumbles.get(FEEDER_RUN_RUMBLE_INDEX)),
        };

        private final Runnable[] profileArray = new Runnable[4];
        private static int currentProfileIndex = 0;

        // =============================================================================================================
        // PathPlanner
        // =============================================================================================================
        private final SendableChooser<Command> autoChooser;
        private final Map<String, Command> eventsAuto;

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
                profileArray[0] = this::defaultBindingsProfile;
                profileArray[1] = this::leftClawBindingsProfile;
                profileArray[2] = this::doubleClawBindingsProfile;
                profileArray[3] = this::rightClawBindingsProfile;
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
                                (speeds) -> drivetrain.applyRequest(() -> robotDrive
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

                // Event map: For PathPlanner markers (e.g., "shoot" triggers shooter)
                eventsAuto = new HashMap<>();
                eventsAuto.put(EVENT_SHOOT,
                                new LockOnShootAndDrive(shooter, drivetrain, loader, aimCamera, null, null,
                                                () -> powerDistribution.getVoltage(),
                                                robotConfig.moduleConfig.maxDriveVelocityMPS));
                eventsAuto.put(EVENT_COLLECT, collector.run(() -> 0.5));
                eventsAuto.put(EVENT_CLIMB, new LockOnClimb(climber, drivetrain, aimCamera, MaxSpeed, MaxAngularRate));

                // Setup the auto UI in Shuffleboard.
                autoChooser = AutoBuilder.buildAutoChooser();
                ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
                autoTab.add("Auto Chooser", autoChooser).withWidget("ComboBox Chooser");

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
                drivetrain.setDefaultCommand(commands[NORMAL_DRIVE_INDEX]);
                RobotModeTriggers.disabled().whileTrue(commands[IDLE_INDEX]);
                controller.back().onTrue(commands[PROFILE_DECREASE]);
                controller.start().onTrue(commands[PROFILE_INCREASE]);
        }

        // -------------------------------------------------------------------------------------------------------------
        private void defaultBindingsProfile() {
                setDefaultBindings();
                rumbles.set(COLLECTOR_RUN_RUMBLE_INDEX, () -> RumbleType.kLeftRumble);
                rumbles.set(MANUAL_SHOOT_RUMBLE_INDEX, () -> RumbleType.kRightRumble);
                rumbles.set(BRAKE_RUMBLE_INDEX, () -> RumbleType.kBothRumble);
                rumbles.set(FEEDER_RUN_RUMBLE_INDEX, () -> RumbleType.kLeftRumble);
                rumbles.set(WHEEL_POINT_RUMBLE_INDEX, () -> RumbleType.kLeftRumble);
                rumbles.set(WEAPON_SWAP_RUMBLE_INDEX, () -> RumbleType.kRightRumble);
                rumbles.set(RESET_FIELD_ORIENTATION_RUMBLE_INDEX, () -> RumbleType.kBothRumble);
                rumbles.set(CLIMBER_UP_RUMBLE_INDEX, () -> RumbleType.kLeftRumble);
                rumbles.set(CLIMBER_DOWN_RUMBLE_INDEX, () -> RumbleType.kRightRumble);
                rumbles.set(HOPPER_RUN_RUMBLE_INDEX, () -> RumbleType.kLeftRumble);

                Command collectorRun = makeCollectorRunCommand(() -> -controller.getLeftTriggerAxis(),
                                rumbles.get(COLLECTOR_RUN_RUMBLE_INDEX));
                commands[COLLECTOR_RUN_INDEX] = collectorRun;
                Command manShoot = makeManualShootCommand(() -> controller.getRightTriggerAxis(),
                                rumbles.get(MANUAL_SHOOT_RUMBLE_INDEX));
                commands[MANUAL_SHOOT_INDEX] = manShoot;

                controller.leftBumper().whileTrue(commands[BRAKE_INDEX]);
                controller.a().and(() -> checkAimbotStatus == false).whileTrue(commands[FEEDER_RUN_INDEX]);
                controller.x().toggleOnTrue(commands[WEAPON_SWAP_INDEX]);
                controller.leftTrigger().onTrue(commands[COLLECTOR_RUN_INDEX]);
                controller.y().onTrue(commands[RESET_FIELD_ORIENTATION_INDEX]);
                controller.povUp().onTrue(commands[CLIMBER_UP_INDEX]);
                controller.povDown().onTrue(commands[CLIMBER_DOWN_INDEX]);
                controller.rightTrigger().and(() -> checkAimbotStatus == false).whileTrue(commands[MANUAL_SHOOT_INDEX]);
                controller.povLeft().onTrue(commands[WHEEL_POINT_INDEX]);
                controller.b().onTrue(commands[HOPPER_RUN_INDEX]);
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
        private void leftClawBindingsProfile() {
                setDefaultBindings();

                rumbles.set(COLLECTOR_RUN_RUMBLE_INDEX, () -> RumbleType.kRightRumble);
                rumbles.set(MANUAL_SHOOT_RUMBLE_INDEX, () -> RumbleType.kLeftRumble);
                rumbles.set(BRAKE_RUMBLE_INDEX, () -> RumbleType.kRightRumble);
                rumbles.set(FEEDER_RUN_RUMBLE_INDEX, () -> RumbleType.kLeftRumble);
                rumbles.set(WHEEL_POINT_RUMBLE_INDEX, () -> RumbleType.kLeftRumble);
                rumbles.set(WEAPON_SWAP_RUMBLE_INDEX, () -> RumbleType.kLeftRumble);
                rumbles.set(RESET_FIELD_ORIENTATION_RUMBLE_INDEX, () -> RumbleType.kLeftRumble);
                rumbles.set(CLIMBER_UP_RUMBLE_INDEX, () -> RumbleType.kRightRumble);
                rumbles.set(CLIMBER_DOWN_RUMBLE_INDEX, () -> RumbleType.kRightRumble);
                rumbles.set(HOPPER_RUN_RUMBLE_INDEX, () -> RumbleType.kLeftRumble);

                Command collectorRun = makeCollectorRunCommand(() -> -controller.getRightTriggerAxis(),
                                rumbles.get(COLLECTOR_RUN_RUMBLE_INDEX));
                commands[COLLECTOR_RUN_INDEX] = collectorRun;
                Command manShootLeft = makeManualShootCommand(() -> controller.getLeftTriggerAxis(),
                                rumbles.get(MANUAL_SHOOT_RUMBLE_INDEX));
                commands[MANUAL_SHOOT_INDEX] = manShootLeft;

                controller.rightTrigger().onTrue(commands[COLLECTOR_RUN_INDEX]);
                controller.y().whileTrue(commands[CLIMBER_UP_INDEX]);
                controller.a().whileTrue(commands[CLIMBER_DOWN_INDEX]);
                controller.rightTrigger().and(() -> checkAimbotStatus == false).whileTrue(commands[MANUAL_SHOOT_INDEX]);
                controller.povLeft().onTrue(commands[WEAPON_SWAP_INDEX]);
                controller.x().toggleOnTrue(commands[HOPPER_RUN_INDEX]);
                controller.rightBumper().whileTrue(commands[BRAKE_INDEX]);
                controller.b().onTrue(commands[WHEEL_POINT_INDEX]);
                controller.povDown().onTrue(commands[RESET_FIELD_ORIENTATION_INDEX]);
        }

        // -------------------------------------------------------------------------------------------------------------
        private void doubleClawBindingsProfile() {
                setDefaultBindings();

                rumbles.set(COLLECTOR_RUN_RUMBLE_INDEX, () -> RumbleType.kLeftRumble);
                rumbles.set(MANUAL_SHOOT_RUMBLE_INDEX, () -> RumbleType.kRightRumble);
                rumbles.set(BRAKE_RUMBLE_INDEX, () -> RumbleType.kLeftRumble);
                rumbles.set(FEEDER_RUN_RUMBLE_INDEX, () -> RumbleType.kRightRumble);
                rumbles.set(WHEEL_POINT_RUMBLE_INDEX, () -> RumbleType.kLeftRumble);
                rumbles.set(WEAPON_SWAP_RUMBLE_INDEX, () -> RumbleType.kRightRumble);
                rumbles.set(RESET_FIELD_ORIENTATION_RUMBLE_INDEX, () -> RumbleType.kRightRumble);
                rumbles.set(CLIMBER_UP_RUMBLE_INDEX, () -> RumbleType.kLeftRumble);
                rumbles.set(CLIMBER_DOWN_RUMBLE_INDEX, () -> RumbleType.kRightRumble);
                rumbles.set(HOPPER_RUN_RUMBLE_INDEX, () -> RumbleType.kLeftRumble);

                Command CollectorVarDouble = makeCollectorRunCommand(() -> -controller.getLeftTriggerAxis(),
                                rumbles.get(COLLECTOR_RUN_RUMBLE_INDEX));
                commands[COLLECTOR_RUN_INDEX] = CollectorVarDouble;
                Command manShootDouble = makeManualShootCommand(() -> controller.getRightTriggerAxis(),
                                rumbles.get(MANUAL_SHOOT_RUMBLE_INDEX));
                commands[MANUAL_SHOOT_INDEX] = manShootDouble;

                controller.leftTrigger().onTrue(commands[COLLECTOR_RUN_INDEX]);
                controller.povUp().whileTrue(commands[CLIMBER_UP_INDEX]);
                controller.povDown().whileTrue(commands[CLIMBER_DOWN_INDEX]);
                controller.rightTrigger().and(() -> checkAimbotStatus == false).whileTrue(commands[MANUAL_SHOOT_INDEX]);
                controller.y().onTrue(commands[WEAPON_SWAP_INDEX]);
                controller.povRight().whileTrue(commands[BRAKE_INDEX]);
                controller.povLeft().onTrue(commands[WHEEL_POINT_INDEX]);
                controller.a().onTrue(commands[RESET_FIELD_ORIENTATION_INDEX]);
                controller.rightBumper().onTrue(commands[HOPPER_RUN_INDEX]);
                controller.b().toggleOnTrue(commands[HOPPER_RUN_INDEX]);
        }

        // -------------------------------------------------------------------------------------------------------------
        private void rightClawBindingsProfile() {
                setDefaultBindings();

                rumbles.set(COLLECTOR_RUN_RUMBLE_INDEX, () -> RumbleType.kLeftRumble);
                rumbles.set(MANUAL_SHOOT_RUMBLE_INDEX, () -> RumbleType.kRightRumble);
                rumbles.set(BRAKE_RUMBLE_INDEX, () -> RumbleType.kLeftRumble);
                rumbles.set(FEEDER_RUN_RUMBLE_INDEX, () -> RumbleType.kRightRumble);
                rumbles.set(WHEEL_POINT_RUMBLE_INDEX, () -> RumbleType.kLeftRumble);
                rumbles.set(WEAPON_SWAP_RUMBLE_INDEX, () -> RumbleType.kRightRumble);
                rumbles.set(RESET_FIELD_ORIENTATION_RUMBLE_INDEX, () -> RumbleType.kRightRumble);
                rumbles.set(CLIMBER_UP_RUMBLE_INDEX, () -> RumbleType.kLeftRumble);
                rumbles.set(CLIMBER_DOWN_RUMBLE_INDEX, () -> RumbleType.kLeftRumble);
                rumbles.set(HOPPER_RUN_RUMBLE_INDEX, () -> RumbleType.kLeftRumble);

                Command CollectorVarRight = makeCollectorRunCommand(() -> -controller.getLeftTriggerAxis(),
                                rumbles.get(COLLECTOR_RUN_RUMBLE_INDEX));
                commands[COLLECTOR_RUN_INDEX] = CollectorVarRight;
                Command manShootRight = makeManualShootCommand(() -> controller.getRightTriggerAxis(),
                                rumbles.get(MANUAL_SHOOT_RUMBLE_INDEX));
                commands[MANUAL_SHOOT_INDEX] = manShootRight;

                controller.leftTrigger().onTrue(commands[COLLECTOR_RUN_INDEX]);
                controller.povUp().whileTrue(commands[CLIMBER_UP_INDEX]);
                controller.povDown().whileTrue(commands[CLIMBER_DOWN_INDEX]);
                controller.rightTrigger().and(() -> checkAimbotStatus == false).whileTrue(commands[MANUAL_SHOOT_INDEX]);
                controller.y().onTrue(commands[WEAPON_SWAP_INDEX]);
                controller.leftBumper().whileTrue(commands[BRAKE_INDEX]);
                controller.povLeft().onTrue(commands[WHEEL_POINT_INDEX]);
                controller.a().onTrue(commands[RESET_FIELD_ORIENTATION_INDEX]);
                controller.b().toggleOnTrue(commands[HOPPER_RUN_INDEX]);
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
        private Command makeBrakeCommand(final Supplier<RumbleType> side) {
                return new ParallelCommandGroup(drivetrain.applyRequest(() -> brake),
                                RumblePulseCommand.createLongSinglePulse(controller, RumbleIntensity.MEDIUM_LIGHT,
                                                side));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeWheelsPointCommand(final Supplier<RumbleType> side) {
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
                                                side));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeLockOnShootAndDriveCommand() {
                return new LockOnShootAndDrive(
                                shooter,
                                drivetrain,
                                loader,
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
        private Command makeManualShootCommand(final DoubleSupplier ballVelocityScalar,
                        final Supplier<RumbleType> side) {
                return new ParallelCommandGroup(shooter.manualShootBall(ballVelocityScalar),
                                new RumbleDynamicCommand(controller, ballVelocityScalar, side));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeManualFeederCommand(final Supplier<RumbleType> side) {
                return new ParallelCommandGroup(
                                loader.manualFeederRun(),
                                new RumbleDynamicCommand(controller, () -> RumbleIntensity.MEDIUM, side));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeWeaponSwapCommand(final Supplier<RumbleType> side) {
                final Command weaponSwap = new InstantCommand(() -> {
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

                return new ParallelCommandGroup(weaponSwap,
                                RumblePulseCommand.createShortSinglePulse(controller, RumbleIntensity.SUPER_HEAVY,
                                                side));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeCollectorRunCommand(final DoubleSupplier collectorScalar, final Supplier<RumbleType> side) {
                return new ParallelCommandGroup(collector.run(collectorScalar),
                                new RumbleDynamicCommand(controller, collectorScalar, side));

        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeResetFieldOrientationCommand(final Supplier<RumbleType> side) {
                // Reset the field-centric heading on left bumper press.
                return new ParallelCommandGroup(drivetrain.runOnce(drivetrain::seedFieldCentric),
                                RumblePulseCommand.createLongDoublePulse(controller, RumbleIntensity.MEDIUM_HEAVY,
                                                side));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeProfileIncreaseCommand(final Supplier<RumbleType> side) {
                return new InstantCommand(() -> {
                        if (currentProfileIndex == profileArray.length + 1) {
                                currentProfileIndex = -1;
                        }
                        currentProfileIndex = currentProfileIndex + 1;
                        for (int i = 0; i < commands.length; i++) {
                                if (i == PROFILE_DECREASE || i == PROFILE_INCREASE) {
                                        continue;
                                }
                                getCommandScheduler().cancel(commands[i]);
                        }
                        getCommandScheduler().getActiveButtonLoop().clear();
                        final Runnable profile = profileArray[currentProfileIndex];
                        profile.run();
                });
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeProfileDecreaseCommand(final Supplier<RumbleType> side) {
                return new InstantCommand(() -> {
                        if (currentProfileIndex == 0) {
                                currentProfileIndex = profileArray.length;
                        }
                        currentProfileIndex = currentProfileIndex - 1;
                        for (int i = 0; i < commands.length; i++) {
                                if (i == PROFILE_DECREASE || i == PROFILE_INCREASE) {
                                        continue;
                                }
                                getCommandScheduler().cancel(commands[i]);
                        }
                        getCommandScheduler().getActiveButtonLoop().clear();
                        final Runnable profile = profileArray[currentProfileIndex];
                        profile.run();
                });
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

        // -------------------------------------------------------------------------------------------------------------
        private Command makeClimberUpCommand(final Supplier<RumbleType> side) {
                return new ParallelCommandGroup(climber.upward(),
                                RumblePulseCommand.createShortDoublePulse(controller, RumbleIntensity.MEDIUM,
                                                side));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeClimberDownCommand(final Supplier<RumbleType> side) {
                return new ParallelCommandGroup(climber.downward(),
                                new SequentialCommandGroup(
                                                RumblePulseCommand.createShortDoublePulse(controller,
                                                                RumbleIntensity.MEDIUM,
                                                                side),
                                                RumblePulseCommand.createShortWaitCommand(),
                                                RumblePulseCommand.createShortSinglePulse(controller,
                                                                RumbleIntensity.MEDIUM,
                                                                side)));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeHopperRunCommand(final Supplier<RumbleType> side) {
                return new ParallelCommandGroup(new SequentialCommandGroup(
                                hopper.unclasp(),
                                new WaitCommand(0.5),
                                hopper.stop()),
                                RumblePulseCommand.createLongSinglePulse(controller, RumbleIntensity.LIGHT,
                                                side));
        }
}