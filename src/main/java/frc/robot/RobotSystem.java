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
import java.util.function.Consumer;
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
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.climb.LockOnClimb;
import frc.robot.commands.rumble.RumbleDynamicCommand;
import frc.robot.commands.rumble.RumbleIntensity;
import frc.robot.commands.rumble.RumblePulseCommand;
import frc.robot.commands.shoot.LockOnShootAndDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.vision.AimCamera;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Climber;

public class RobotSystem {
        // =============================================================================================================
        // Constants
        // =============================================================================================================
        // kSpeedAt12Volts desired top speed
        private static final double MaxSpeed = 1 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        // 3/4 of a rotation per second max angular velocity
        private static final double MaxAngularRateScaler = 0.75;
        private static final double MaxAngularRate = RotationsPerSecond.of(MaxAngularRateScaler).in(RadiansPerSecond);

        private static final String EVENT_SHOOT = "shoot";
        private static final String EVENT_COLLECT = "collect";
        private static final String EVENT_CLIMB = "climb";

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
        private final Agitator agitator = new Agitator();
        private final Climber climber = new Climber();

        // =============================================================================================================
        // Modules
        // =============================================================================================================
        private final PowerDistribution powerDistribution = new PowerDistribution(CANBus.ID.POWER_DISTRIBUTION,
                        ModuleType.kRev);

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
        private static final byte CLIMBER_UP_INDEX = 7;
        private static final byte CLIMBER_DOWN_INDEX = 8;
        private static final byte FEEDER_RUN_OUT_INDEX = 9;
        private static final byte SPEED_CHANGE_INDEX = 10;
        private static final byte HOPPER_IN_INDEX = 11;
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
                        /* Climber Up */
                        null,
                        /* Climber Down */
                        null,
                        /* Manual Feeder Out */
                        null,
                        /* Speed Changing */
                        null,
                        /* Hopper In */
                        null,
        };
        private static int currentProfileIndex = 0;
        private final List<Consumer<EventLoop>> profiles;
        private final List<EventLoop> profileEventLoops;

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

                /*
                 * IMPORTANT!
                 * 
                 * This setDefaultCommand method's behavior is not tied to the event loop and
                 * can therefore be set outside the profiles. Plus, it never changes.
                 */
                drivetrain.setDefaultCommand(makeNormalDriveCommand(driver));

                this.profiles = List.of(
                                profile1 -> defaultBindingsProfile(profile1),
                                profile2 -> leftClawBindingsProfile(profile2),
                                profile3 -> doubleClawBindingsProfile(profile3),
                                profile4 -> rightClawBindingsProfile(profile4));

                this.profileEventLoops = new ArrayList<>(profiles.size());
                for (int i = 0; i < profiles.size(); i++) {
                        EventLoop loop = new EventLoop();
                        profiles.get(i).accept(loop);
                        profileEventLoops.add(loop);
                }
                getCommandScheduler().setActiveButtonLoop(profileEventLoops.get(0));

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
                                new LockOnShootAndDrive(shooter, drivetrain, feeder, aimCamera, null, null,
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
        private void setDefaultBindings(final EventLoop profile) {
                new Trigger(profile, () -> driver.start().getAsBoolean()).onTrue(makeProfileIncreaseCommand(driver));
                new Trigger(profile, () -> driver.back().getAsBoolean()).onTrue(makeProfileDecreaseCommand(driver));
                new Trigger(profile, DriverStation::isDisabled).whileTrue(makeIdleCommand());
        }

        // -------------------------------------------------------------------------------------------------------------
        private void defaultBindingsProfile(final EventLoop profile) {
                setDefaultBindings(profile);

                commands[COLLECTOR_RUN_INDEX] = makeCollectorRunCommand(() -> -driver.getLeftTriggerAxis(),
                                () -> RumbleType.kLeftRumble, driver);
                commands[MANUAL_SHOOT_INDEX] = makeManualShootCommand(() -> driver.getRightTriggerAxis(),
                                () -> RumbleType.kRightRumble, driver);
                commands[HOPPER_IN_INDEX] = new ParallelCommandGroup(
                                makeManualFeederInCommand(() -> RumbleType.kLeftRumble, driver),
                                makeAgitatorRunCommand(() -> RumbleType.kRightRumble, driver),
                                RumblePulseCommand.createShortDoublePulse(driver, RumbleIntensity.MEDIUM,
                                                () -> RumbleType.kRightRumble)
                                                .handleInterrupt(() -> driver.setRumble(RumbleType.kRightRumble, 0)));
                commands[BRAKE_INDEX] = makeBrakeCommand(() -> RumbleType.kBothRumble, driver);
                commands[WHEEL_POINT_INDEX] = makeWheelsPointCommand(() -> RumbleType.kLeftRumble, driver);
                commands[LOCK_ON_SHOOT_AND_DRIVE_INDEX] = makeLockOnShootAndDriveCommand(() -> RumbleType.kBothRumble,
                                driver);
                commands[RESET_FIELD_ORIENTATION_INDEX] = makeResetFieldOrientationCommand(
                                () -> RumbleType.kBothRumble, driver);
                commands[WEAPON_SWAP_INDEX] = makeWeaponSwapCommand(() -> RumbleType.kRightRumble, driver);
                commands[CLIMBER_UP_INDEX] = makeClimberUpCommand(() -> RumbleType.kLeftRumble, driver);
                commands[CLIMBER_DOWN_INDEX] = makeClimberDownCommand(() -> RumbleType.kRightRumble, driver);
                commands[FEEDER_RUN_OUT_INDEX] = makeManualFeederOutCommand(() -> RumbleType.kLeftRumble, driver);
                commands[SPEED_CHANGE_INDEX] = makeMaxSpeedChangeCommand(() -> RumbleType.kBothRumble, driver);

                new Trigger(profile, () -> driver.leftBumper().getAsBoolean()).whileTrue(commands[BRAKE_INDEX]);
                new Trigger(profile, () -> driver.x().getAsBoolean()).toggleOnTrue(commands[WEAPON_SWAP_INDEX]);
                new Trigger(profile, () -> driver.leftTrigger().getAsBoolean())
                                .onTrue(commands[COLLECTOR_RUN_INDEX]);
                new Trigger(profile, () -> driver.y().getAsBoolean())
                                .onTrue(commands[RESET_FIELD_ORIENTATION_INDEX]);
                new Trigger(profile, () -> driver.povUp().getAsBoolean()).onTrue(commands[CLIMBER_UP_INDEX]);
                new Trigger(profile, () -> driver.povDown().getAsBoolean()).onTrue(commands[CLIMBER_DOWN_INDEX]);
                new Trigger(profile, () -> driver.rightTrigger().getAsBoolean())
                                .and(() -> isLockedOn == false)
                                .whileTrue(commands[MANUAL_SHOOT_INDEX]);
                new Trigger(profile, () -> driver.povLeft().getAsBoolean()).onTrue(commands[WHEEL_POINT_INDEX]);
                new Trigger(profile, () -> driver.a().getAsBoolean()).and(() -> isLockedOn == false)
                                .and(() -> driver.rightBumper().getAsBoolean())
                                .whileTrue(commands[FEEDER_RUN_OUT_INDEX]);
                new Trigger(profile, () -> driver.a().getAsBoolean()).whileTrue(commands[HOPPER_IN_INDEX]);
                new Trigger(profile, () -> driver.b().getAsBoolean())
                                .onTrue(commands[SPEED_CHANGE_INDEX]);

        }

        // -------------------------------------------------------------------------------------------------------------
        private void leftClawBindingsProfile(final EventLoop profile) {
                setDefaultBindings(profile);

                commands[COLLECTOR_RUN_INDEX] = makeCollectorRunCommand(() -> -driver.getRightTriggerAxis(),
                                () -> RumbleType.kRightRumble, driver);
                commands[MANUAL_SHOOT_INDEX] = makeManualShootCommand(() -> driver.getLeftTriggerAxis(),
                                () -> RumbleType.kLeftRumble, driver);
                commands[BRAKE_INDEX] = makeBrakeCommand(() -> RumbleType.kRightRumble, driver);
                commands[WHEEL_POINT_INDEX] = makeWheelsPointCommand(() -> RumbleType.kLeftRumble, driver);
                commands[LOCK_ON_SHOOT_AND_DRIVE_INDEX] = makeLockOnShootAndDriveCommand(() -> RumbleType.kBothRumble,
                                driver);
                commands[RESET_FIELD_ORIENTATION_INDEX] = makeResetFieldOrientationCommand(
                                () -> RumbleType.kLeftRumble, driver);
                commands[WEAPON_SWAP_INDEX] = makeWeaponSwapCommand(() -> RumbleType.kLeftRumble, driver);
                commands[CLIMBER_UP_INDEX] = makeClimberUpCommand(() -> RumbleType.kRightRumble, driver);
                commands[CLIMBER_DOWN_INDEX] = makeClimberDownCommand(() -> RumbleType.kRightRumble, driver);

                new Trigger(profile, () -> driver.rightTrigger().getAsBoolean())
                                .onTrue(commands[COLLECTOR_RUN_INDEX]);
                new Trigger(profile, () -> driver.y().getAsBoolean()).whileTrue(commands[CLIMBER_UP_INDEX]);
                new Trigger(profile, () -> driver.a().getAsBoolean()).whileTrue(commands[CLIMBER_DOWN_INDEX]);
                new Trigger(profile, () -> driver.leftTrigger().getAsBoolean())
                                .and(() -> isLockedOn == false)
                                .whileTrue(commands[MANUAL_SHOOT_INDEX]);
                new Trigger(profile, () -> driver.povLeft().getAsBoolean()).onTrue(commands[WEAPON_SWAP_INDEX]);
                new Trigger(profile, () -> driver.x().getAsBoolean()).whileTrue(commands[BRAKE_INDEX]);
                new Trigger(profile, () -> driver.b().getAsBoolean()).onTrue(commands[WHEEL_POINT_INDEX]);
                new Trigger(profile, () -> driver.povDown().getAsBoolean())
                                .onTrue(commands[RESET_FIELD_ORIENTATION_INDEX]);
                new Trigger(profile, () -> driver.povRight().getAsBoolean()).and(() -> isLockedOn == false)
                                .and(() -> driver.rightBumper().getAsBoolean())
                                .whileTrue(commands[FEEDER_RUN_OUT_INDEX]);
        }

        // -------------------------------------------------------------------------------------------------------------
        private void doubleClawBindingsProfile(final EventLoop profile) {
                setDefaultBindings(profile);

                commands[BRAKE_INDEX] = makeBrakeCommand(() -> RumbleType.kLeftRumble, driver);
                commands[WHEEL_POINT_INDEX] = makeWheelsPointCommand(() -> RumbleType.kLeftRumble, driver);
                commands[LOCK_ON_SHOOT_AND_DRIVE_INDEX] = makeLockOnShootAndDriveCommand(
                                () -> RumbleType.kBothRumble, driver);
                commands[RESET_FIELD_ORIENTATION_INDEX] = makeResetFieldOrientationCommand(
                                () -> RumbleType.kRightRumble, driver);
                commands[WEAPON_SWAP_INDEX] = makeWeaponSwapCommand(() -> RumbleType.kRightRumble, driver);
                commands[CLIMBER_UP_INDEX] = makeClimberUpCommand(() -> RumbleType.kLeftRumble, driver);
                commands[CLIMBER_DOWN_INDEX] = makeClimberDownCommand(() -> RumbleType.kRightRumble, driver);
                commands[COLLECTOR_RUN_INDEX] = makeCollectorRunCommand(() -> -driver.getLeftTriggerAxis(),
                                () -> RumbleType.kLeftRumble, driver);

                commands[MANUAL_SHOOT_INDEX] = makeManualShootCommand(() -> driver.getRightTriggerAxis(),
                                () -> RumbleType.kRightRumble, driver);

                new Trigger(profile, () -> driver.leftTrigger().getAsBoolean())
                                .onTrue(commands[COLLECTOR_RUN_INDEX]);
                new Trigger(profile, () -> driver.povUp().getAsBoolean()).whileTrue(commands[CLIMBER_UP_INDEX]);
                new Trigger(profile, () -> driver.povDown().getAsBoolean()).whileTrue(commands[CLIMBER_DOWN_INDEX]);
                new Trigger(profile, () -> driver.rightTrigger().getAsBoolean())
                                .and(() -> isLockedOn == false)
                                .whileTrue(commands[MANUAL_SHOOT_INDEX]);
                new Trigger(profile, () -> driver.y().getAsBoolean()).onTrue(commands[WEAPON_SWAP_INDEX]);
                new Trigger(profile, () -> driver.povRight().getAsBoolean()).whileTrue(commands[BRAKE_INDEX]);
                new Trigger(profile, () -> driver.povLeft().getAsBoolean()).onTrue(commands[WHEEL_POINT_INDEX]);
                new Trigger(profile, () -> driver.a().getAsBoolean())
                                .onTrue(commands[RESET_FIELD_ORIENTATION_INDEX]);
                new Trigger(profile, () -> driver.x().getAsBoolean()).and(() -> isLockedOn == false)
                                .and(() -> driver.rightBumper().getAsBoolean())
                                .whileTrue(commands[FEEDER_RUN_OUT_INDEX]);
        }

        // -------------------------------------------------------------------------------------------------------------
        private void rightClawBindingsProfile(final EventLoop profile) {
                setDefaultBindings(profile);

                commands[BRAKE_INDEX] = makeBrakeCommand(() -> RumbleType.kLeftRumble, driver);
                commands[WHEEL_POINT_INDEX] = makeWheelsPointCommand(() -> RumbleType.kLeftRumble, driver);
                commands[LOCK_ON_SHOOT_AND_DRIVE_INDEX] = makeLockOnShootAndDriveCommand(
                                () -> RumbleType.kBothRumble, driver);
                commands[RESET_FIELD_ORIENTATION_INDEX] = makeResetFieldOrientationCommand(
                                () -> RumbleType.kRightRumble, driver);
                commands[WEAPON_SWAP_INDEX] = makeWeaponSwapCommand(() -> RumbleType.kRightRumble, driver);
                commands[CLIMBER_UP_INDEX] = makeClimberUpCommand(() -> RumbleType.kLeftRumble, driver);
                commands[CLIMBER_DOWN_INDEX] = makeClimberDownCommand(() -> RumbleType.kLeftRumble, driver);
                commands[COLLECTOR_RUN_INDEX] = makeCollectorRunCommand(() -> -driver.getLeftTriggerAxis(),
                                () -> RumbleType.kLeftRumble, driver);
                commands[MANUAL_SHOOT_INDEX] = makeManualShootCommand(() -> driver.getRightTriggerAxis(),
                                () -> RumbleType.kRightRumble, driver);

                new Trigger(profile, () -> driver.leftTrigger().getAsBoolean())
                                .onTrue(commands[COLLECTOR_RUN_INDEX]);
                new Trigger(profile, () -> driver.povUp().getAsBoolean()).whileTrue(commands[CLIMBER_UP_INDEX]);
                new Trigger(profile, () -> driver.povDown().getAsBoolean()).whileTrue(commands[CLIMBER_DOWN_INDEX]);
                new Trigger(profile, () -> driver.rightTrigger().getAsBoolean())
                                .and(() -> isLockedOn == false)
                                .whileTrue(commands[MANUAL_SHOOT_INDEX]);
                new Trigger(profile, () -> driver.y().getAsBoolean()).onTrue(commands[WEAPON_SWAP_INDEX]);
                new Trigger(profile, () -> driver.leftBumper().getAsBoolean()).whileTrue(commands[BRAKE_INDEX]);
                new Trigger(profile, () -> driver.povLeft().getAsBoolean()).onTrue(commands[WHEEL_POINT_INDEX]);
                new Trigger(profile, () -> driver.a().getAsBoolean())
                                .onTrue(commands[RESET_FIELD_ORIENTATION_INDEX]);
                new Trigger(profile, () -> driver.x().getAsBoolean()).and(() -> isLockedOn == false)
                                .and(() -> driver.rightBumper().getAsBoolean())
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
                                () -> -driver.getLeftX() * MaxSpeed,
                                () -> -driver.getLeftY() * MaxSpeed,
                                () -> powerDistribution.getVoltage(),
                                MaxSpeed)
                                .handleInterrupt(() -> {
                                        System.out.println("I am wondering if this executes on cancel()?");
                                        getCommandScheduler().schedule(RumblePulseCommand
                                                        .createShortDoublePulse(controller,
                                                                        RumbleIntensity.SUPER_HEAVY,
                                                                        side)
                                                        .handleInterrupt(() -> controller
                                                                        .setRumble(side.get(), 0)));
                                        isLockedOn = true;
                                        getCommandScheduler().schedule(commands[MANUAL_SHOOT_INDEX]);
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
                                getCommandScheduler().schedule(commands[MANUAL_SHOOT_INDEX]);
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
        private Command makeProfileIncreaseCommand(final CommandXboxController controller) {
                return new InstantCommand(() -> {
                        System.out.println("Profile Increase");
                        System.out.println("profiles.size() = " + profiles.size());
                        System.out.println("profileEventLoops.size() = " + profileEventLoops.size());
                        System.out.println("currentProfileInex = " + currentProfileIndex);
                        currentProfileIndex = currentProfileIndex + 1;
                        if (currentProfileIndex == profileEventLoops.size()) {
                                currentProfileIndex = 0;
                        }
                        getCommandScheduler().setActiveButtonLoop(profileEventLoops.get(currentProfileIndex));
                        for (int i = 0; i < commands.length; i++) {
                                getCommandScheduler().cancel(commands[i]);
                        }
                        getCommandScheduler().schedule(
                                        new RumblePulseCommand(controller,
                                                        0.3,
                                                        0.5,
                                                        RumbleIntensity.SUPER_HEAVY,
                                                        (byte) (currentProfileIndex + 1),
                                                        () -> RumbleType.kBothRumble)
                                                        .handleInterrupt(() -> controller
                                                                        .setRumble(RumbleType.kBothRumble, 0)));
                });
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeProfileDecreaseCommand(final CommandXboxController controller) {
                return new InstantCommand(() -> {
                        System.out.println("Profile Decrease");
                        System.out.println("profiles.size() = " + profiles.size());
                        System.out.println("profileEventLoops.size() = " + profileEventLoops.size());
                        System.out.println("currentProfileInex = " + currentProfileIndex);
                        currentProfileIndex = currentProfileIndex - 1;
                        if (currentProfileIndex <= 0) {
                                currentProfileIndex = profileEventLoops.size() - 1;
                        }
                        getCommandScheduler().setActiveButtonLoop(profileEventLoops.get(currentProfileIndex));
                        for (int i = 0; i < commands.length; i++) {
                                getCommandScheduler().cancel(commands[i]);
                        }

                        getCommandScheduler().schedule(
                                        new RumblePulseCommand(controller,
                                                        0.3,
                                                        0.5,
                                                        RumbleIntensity.SUPER_HEAVY,
                                                        (byte) (currentProfileIndex + 1),
                                                        () -> RumbleType.kBothRumble)
                                                        .handleInterrupt(() -> controller
                                                                        .setRumble(RumbleType.kBothRumble, 0)));
                });
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeClimberUpCommand(final Supplier<RumbleType> side, final CommandXboxController controller) {
                return new ParallelCommandGroup(climber.upward(),
                                RumblePulseCommand.createShortDoublePulse(controller, RumbleIntensity.MEDIUM,
                                                side).handleInterrupt(() -> controller.setRumble(side.get(), 0)));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeClimberDownCommand(final Supplier<RumbleType> side,
                        final CommandXboxController controller) {
                return new ParallelCommandGroup(climber.downward(),
                                new SequentialCommandGroup(
                                                RumblePulseCommand.createShortDoublePulse(controller,
                                                                RumbleIntensity.MEDIUM,
                                                                side)
                                                                .handleInterrupt(() -> controller.setRumble(side.get(),
                                                                                0)),
                                                RumblePulseCommand.createShortWaitCommand()
                                                                .handleInterrupt(() -> controller.setRumble(side.get(),
                                                                                0)),
                                                RumblePulseCommand.createShortSinglePulse(controller,
                                                                RumbleIntensity.MEDIUM,
                                                                side)
                                                                .handleInterrupt(() -> controller.setRumble(side.get(),
                                                                                0)))
                                                .handleInterrupt(() -> controller.setRumble(side.get(),
                                                                0)));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeAgitatorRunCommand(final Supplier<RumbleType> side,
                        final CommandXboxController controller) {
                return new ParallelCommandGroup(agitator.run(),
                                RumblePulseCommand.createLongDoublePulse(controller, RumbleIntensity.MEDIUM, side)
                                                .handleInterrupt(() -> controller.setRumble(side.get(), 0)));
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