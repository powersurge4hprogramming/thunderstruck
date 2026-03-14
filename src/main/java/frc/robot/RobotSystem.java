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
import frc.robot.subsystems.Aggitator;
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
        private final CommandXboxController driver = new CommandXboxController(USB.CONTROLLER.DRIVER);
        private final CommandXboxController operator = new CommandXboxController(USB.CONTROLLER.OPERATOR);
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
        private final Feeder feeder = new Feeder();
        private final Collector collector = new Collector();
        private final Aggitator aggitator = new Aggitator();
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
        @SuppressWarnings("unused")
        private static final byte SYSID_DYNAMIC_FORWARD_INDEX = 6;
        @SuppressWarnings("unused")
        private static final byte SYSID_DYNAMIC_REVERSE_INDEX = 7;
        @SuppressWarnings("unused")
        private static final byte SYSID_QUASISTATIC_FORWARD_INDEX = 8;
        @SuppressWarnings("unused")
        private static final byte SYSID_QUASISTATIC_REVERSE_INDEX = 9;
        private static final byte WEAPON_SWAP_INDEX = 10;
        private static final byte CLIMBER_UP_INDEX = 11;
        private static final byte CLIMBER_DOWN_INDEX = 12;
        private static final byte FEEDER_RUN_INDEX = 13;
        private static final byte AGGITATOR_RUN_INDEX = 14;
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
                        /*
                         * TODO: I don't think these need to be saved here for cancelling.
                         * 
                         * They can just be mapped in normal mode and not have to be worried about being
                         * cancelled because in reality they won't be running in a profile switched
                         * scenario.
                         */
                        makeSysIdDynamicForwardCommand(),
                        makeSysIdDynamicReverseCommand(),
                        makeSysIdQuasistaticForwardCommand(),
                        makeSysIdQuasistaticReverseCommand(),
                        /* Weapon Swap */
                        null,
                        /* Climber Up */
                        null,
                        /* Climber Down */
                        null,
                        /* Manual Feeder */
                        null,
                        /* Aggitator Run */
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
                drivetrain.setDefaultCommand(makeNormalDriveCommand());

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
                new Trigger(profile, () -> driver.start().getAsBoolean()).onTrue(makeProfileIncreaseCommand());
                new Trigger(profile, () -> driver.back().getAsBoolean()).onTrue(makeProfileDecreaseCommand());
                new Trigger(profile, DriverStation::isDisabled).whileTrue(makeIdleCommand());
        }

        // -------------------------------------------------------------------------------------------------------------
        private void defaultBindingsProfile(final EventLoop profile) {
                setDefaultBindings(profile);

                commands[COLLECTOR_RUN_INDEX] = makeCollectorRunCommand(() -> -driver.getLeftTriggerAxis(),
                                () -> RumbleType.kLeftRumble);
                commands[MANUAL_SHOOT_INDEX] = makeManualShootCommand(() -> driver.getRightTriggerAxis(),
                                () -> RumbleType.kRightRumble);
                commands[BRAKE_INDEX] = makeBrakeCommand(() -> RumbleType.kBothRumble);
                commands[WHEEL_POINT_INDEX] = makeWheelsPointCommand(() -> RumbleType.kLeftRumble);
                commands[LOCK_ON_SHOOT_AND_DRIVE_INDEX] = makeLockOnShootAndDriveCommand(() -> RumbleType.kBothRumble);
                commands[RESET_FIELD_ORIENTATION_INDEX] = makeResetFieldOrientationCommand(
                                () -> RumbleType.kBothRumble);
                commands[WEAPON_SWAP_INDEX] = makeWeaponSwapCommand(() -> RumbleType.kRightRumble);
                commands[CLIMBER_UP_INDEX] = makeClimberUpCommand(() -> RumbleType.kLeftRumble);
                commands[CLIMBER_DOWN_INDEX] = makeClimberDownCommand(() -> RumbleType.kRightRumble);
                commands[FEEDER_RUN_INDEX] = makeManualFeederCommand(() -> RumbleType.kLeftRumble);
                commands[AGGITATOR_RUN_INDEX] = makeAggitorRunCommand(() -> RumbleType.kBothRumble);

                new Trigger(profile, () -> driver.leftBumper().getAsBoolean()).whileTrue(commands[BRAKE_INDEX]);
                new Trigger(profile, () -> driver.a().getAsBoolean()).and(() -> checkAimbotStatus == false)
                                .whileTrue(commands[FEEDER_RUN_INDEX]);
                new Trigger(profile, () -> driver.x().getAsBoolean()).toggleOnTrue(commands[WEAPON_SWAP_INDEX]);
                new Trigger(profile, () -> driver.leftTrigger().getAsBoolean())
                                .onTrue(commands[COLLECTOR_RUN_INDEX]);
                new Trigger(profile, () -> driver.y().getAsBoolean())
                                .onTrue(commands[RESET_FIELD_ORIENTATION_INDEX]);
                new Trigger(profile, () -> driver.povUp().getAsBoolean()).onTrue(commands[CLIMBER_UP_INDEX]);
                new Trigger(profile, () -> driver.povDown().getAsBoolean()).onTrue(commands[CLIMBER_DOWN_INDEX]);
                new Trigger(profile, () -> driver.rightTrigger().getAsBoolean())
                                .and(() -> checkAimbotStatus == false)
                                .whileTrue(commands[MANUAL_SHOOT_INDEX]);
                new Trigger(profile, () -> driver.povLeft().getAsBoolean()).onTrue(commands[WHEEL_POINT_INDEX]);
                new Trigger(profile, () -> driver.povRight().getAsBoolean()).whileTrue(commands[AGGITATOR_RUN_INDEX]);
                /*
                 * CONFLICTING WITH THE PROFILE COMMANDS!
                 * 
                 * Run SysId routines when holding back/start and X/Y. Note that each routine
                 * should be run exactly once in a single log.
                 * controller.back().and(controller.y()).whileTrue(commands[
                 * SYSID_DYNAMIC_FORWARD_INDEX]);
                 * controller.back().and(controller.x()).whileTrue(commands[
                 * SYSID_DYNAMIC_REVERSE_INDEX]);
                 * controller.start().and(controller.y()).whileTrue(commands[
                 * SYSID_QUASISTATIC_FORWARD_INDEX]);
                 * controller.start().and(controller.x()).whileTrue(commands[
                 * SYSID_QUASISTATIC_REVERSE_INDEX]);
                 */
        }

        // -------------------------------------------------------------------------------------------------------------
        private void leftClawBindingsProfile(final EventLoop profile) {
                setDefaultBindings(profile);

                commands[COLLECTOR_RUN_INDEX] = makeCollectorRunCommand(() -> -driver.getRightTriggerAxis(),
                                () -> RumbleType.kRightRumble);
                commands[MANUAL_SHOOT_INDEX] = makeManualShootCommand(() -> driver.getLeftTriggerAxis(),
                                () -> RumbleType.kLeftRumble);
                commands[BRAKE_INDEX] = makeBrakeCommand(() -> RumbleType.kRightRumble);
                commands[WHEEL_POINT_INDEX] = makeWheelsPointCommand(() -> RumbleType.kLeftRumble);
                commands[LOCK_ON_SHOOT_AND_DRIVE_INDEX] = makeLockOnShootAndDriveCommand(() -> RumbleType.kBothRumble);
                commands[RESET_FIELD_ORIENTATION_INDEX] = makeResetFieldOrientationCommand(
                                () -> RumbleType.kLeftRumble);
                commands[WEAPON_SWAP_INDEX] = makeWeaponSwapCommand(() -> RumbleType.kLeftRumble);
                commands[CLIMBER_UP_INDEX] = makeClimberUpCommand(() -> RumbleType.kRightRumble);
                commands[CLIMBER_DOWN_INDEX] = makeClimberDownCommand(() -> RumbleType.kRightRumble);
                commands[FEEDER_RUN_INDEX] = makeManualFeederCommand(() -> RumbleType.kLeftRumble);

                // TODO: This is missing a mapping: 9 of 10.
                new Trigger(profile, () -> driver.rightTrigger().getAsBoolean())
                                .onTrue(commands[COLLECTOR_RUN_INDEX]);
                new Trigger(profile, () -> driver.y().getAsBoolean()).whileTrue(commands[CLIMBER_UP_INDEX]);
                new Trigger(profile, () -> driver.a().getAsBoolean()).whileTrue(commands[CLIMBER_DOWN_INDEX]);
                new Trigger(profile, () -> driver.leftTrigger().getAsBoolean())
                                .and(() -> checkAimbotStatus == false)
                                .whileTrue(commands[MANUAL_SHOOT_INDEX]);
                new Trigger(profile, () -> driver.povLeft().getAsBoolean()).onTrue(commands[WEAPON_SWAP_INDEX]);
                new Trigger(profile, () -> driver.rightBumper().getAsBoolean()).whileTrue(commands[BRAKE_INDEX]);
                new Trigger(profile, () -> driver.b().getAsBoolean()).onTrue(commands[WHEEL_POINT_INDEX]);
                new Trigger(profile, () -> driver.povDown().getAsBoolean())
                                .onTrue(commands[RESET_FIELD_ORIENTATION_INDEX]);
        }

        // -------------------------------------------------------------------------------------------------------------
        private void doubleClawBindingsProfile(final EventLoop profile) {
                setDefaultBindings(profile);

                commands[BRAKE_INDEX] = makeBrakeCommand(() -> RumbleType.kLeftRumble);
                commands[WHEEL_POINT_INDEX] = makeWheelsPointCommand(() -> RumbleType.kLeftRumble);
                commands[LOCK_ON_SHOOT_AND_DRIVE_INDEX] = makeLockOnShootAndDriveCommand(
                                () -> RumbleType.kBothRumble);
                commands[RESET_FIELD_ORIENTATION_INDEX] = makeResetFieldOrientationCommand(
                                () -> RumbleType.kRightRumble);
                commands[WEAPON_SWAP_INDEX] = makeWeaponSwapCommand(() -> RumbleType.kRightRumble);
                commands[CLIMBER_UP_INDEX] = makeClimberUpCommand(() -> RumbleType.kLeftRumble);
                commands[CLIMBER_DOWN_INDEX] = makeClimberDownCommand(() -> RumbleType.kRightRumble);
                commands[FEEDER_RUN_INDEX] = makeManualFeederCommand(() -> RumbleType.kRightRumble);

                commands[COLLECTOR_RUN_INDEX] = makeCollectorRunCommand(() -> -driver.getLeftTriggerAxis(),
                                () -> RumbleType.kLeftRumble);

                commands[MANUAL_SHOOT_INDEX] = makeManualShootCommand(() -> driver.getRightTriggerAxis(),
                                () -> RumbleType.kRightRumble);

                new Trigger(profile, () -> driver.leftTrigger().getAsBoolean())
                                .onTrue(commands[COLLECTOR_RUN_INDEX]);
                new Trigger(profile, () -> driver.povUp().getAsBoolean()).whileTrue(commands[CLIMBER_UP_INDEX]);
                new Trigger(profile, () -> driver.povDown().getAsBoolean()).whileTrue(commands[CLIMBER_DOWN_INDEX]);
                new Trigger(profile, () -> driver.rightTrigger().getAsBoolean())
                                .and(() -> checkAimbotStatus == false)
                                .whileTrue(commands[MANUAL_SHOOT_INDEX]);
                new Trigger(profile, () -> driver.y().getAsBoolean()).onTrue(commands[WEAPON_SWAP_INDEX]);
                new Trigger(profile, () -> driver.povRight().getAsBoolean()).whileTrue(commands[BRAKE_INDEX]);
                new Trigger(profile, () -> driver.povLeft().getAsBoolean()).onTrue(commands[WHEEL_POINT_INDEX]);
                new Trigger(profile, () -> driver.a().getAsBoolean())
                                .onTrue(commands[RESET_FIELD_ORIENTATION_INDEX]);
                // TODO: Are we missing a command?
        }

        // -------------------------------------------------------------------------------------------------------------
        private void rightClawBindingsProfile(final EventLoop profile) {
                setDefaultBindings(profile);

                commands[BRAKE_INDEX] = makeBrakeCommand(() -> RumbleType.kLeftRumble);
                commands[WHEEL_POINT_INDEX] = makeWheelsPointCommand(() -> RumbleType.kLeftRumble);
                commands[LOCK_ON_SHOOT_AND_DRIVE_INDEX] = makeLockOnShootAndDriveCommand(
                                () -> RumbleType.kBothRumble);
                commands[RESET_FIELD_ORIENTATION_INDEX] = makeResetFieldOrientationCommand(
                                () -> RumbleType.kRightRumble);
                commands[WEAPON_SWAP_INDEX] = makeWeaponSwapCommand(() -> RumbleType.kRightRumble);
                commands[CLIMBER_UP_INDEX] = makeClimberUpCommand(() -> RumbleType.kLeftRumble);
                commands[CLIMBER_DOWN_INDEX] = makeClimberDownCommand(() -> RumbleType.kLeftRumble);
                commands[FEEDER_RUN_INDEX] = makeManualFeederCommand(() -> RumbleType.kRightRumble);
                commands[COLLECTOR_RUN_INDEX] = makeCollectorRunCommand(() -> -driver.getLeftTriggerAxis(),
                                () -> RumbleType.kLeftRumble);
                commands[MANUAL_SHOOT_INDEX] = makeManualShootCommand(() -> driver.getRightTriggerAxis(),
                                () -> RumbleType.kRightRumble);

                new Trigger(profile, () -> driver.leftTrigger().getAsBoolean())
                                .onTrue(commands[COLLECTOR_RUN_INDEX]);
                new Trigger(profile, () -> driver.povUp().getAsBoolean()).whileTrue(commands[CLIMBER_UP_INDEX]);
                new Trigger(profile, () -> driver.povDown().getAsBoolean()).whileTrue(commands[CLIMBER_DOWN_INDEX]);
                new Trigger(profile, () -> driver.rightTrigger().getAsBoolean())
                                .and(() -> checkAimbotStatus == false)
                                .whileTrue(commands[MANUAL_SHOOT_INDEX]);
                new Trigger(profile, () -> driver.y().getAsBoolean()).onTrue(commands[WEAPON_SWAP_INDEX]);
                new Trigger(profile, () -> driver.leftBumper().getAsBoolean()).whileTrue(commands[BRAKE_INDEX]);
                new Trigger(profile, () -> driver.povLeft().getAsBoolean()).onTrue(commands[WHEEL_POINT_INDEX]);
                new Trigger(profile, () -> driver.a().getAsBoolean())
                                .onTrue(commands[RESET_FIELD_ORIENTATION_INDEX]);
                // TODO: Missing feeder: this is the tenth mapping.
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
                fieldDrive.withVelocityX(-driver.getLeftY() * MaxSpeed)
                                // Drive left with negative X (left)
                                .withVelocityY(-driver.getLeftX() * MaxSpeed)
                                // Drive counterclockwise with negative X (left)
                                .withRotationalRate(-driver.getRightX() * MaxAngularRate));
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
                                RumblePulseCommand.createLongSinglePulse(driver, RumbleIntensity.MEDIUM_LIGHT,
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
                                RumblePulseCommand.createShortSinglePulse(driver, RumbleIntensity.VERY_LIGHT,
                                                side));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeLockOnShootAndDriveCommand(final Supplier<RumbleType> side) {
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
                                        checkAimbotStatus = true;
                                        getCommandScheduler().schedule(new ParallelCommandGroup(
                                                        commands[MANUAL_SHOOT_INDEX]),
                                                        RumblePulseCommand.createLongDoublePulse(driver,
                                                                        RumbleIntensity.SUPER_HEAVY,
                                                                        side));
                                });
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeManualShootCommand(final DoubleSupplier ballVelocityScalar,
                        final Supplier<RumbleType> side) {
                return new ParallelCommandGroup(shooter.manualShootBall(ballVelocityScalar),
                                new RumbleDynamicCommand(driver, ballVelocityScalar, side));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeManualFeederCommand(final Supplier<RumbleType> side) {
                return new ParallelCommandGroup(
                                feeder.manualFeederRun(),
                                new RumbleDynamicCommand(driver, () -> RumbleIntensity.MEDIUM, side));
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
                                RumblePulseCommand.createShortSinglePulse(driver, RumbleIntensity.SUPER_HEAVY,
                                                side));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeCollectorRunCommand(final DoubleSupplier collectorScalar, final Supplier<RumbleType> side) {
                return new ParallelCommandGroup(collector.run(collectorScalar),
                                new RumbleDynamicCommand(driver, collectorScalar, side));

        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeResetFieldOrientationCommand(final Supplier<RumbleType> side) {
                // Reset the field-centric heading on left bumper press.
                return new ParallelCommandGroup(drivetrain.runOnce(drivetrain::seedFieldCentric),
                                RumblePulseCommand.createLongDoublePulse(driver, RumbleIntensity.MEDIUM_HEAVY,
                                                side));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeProfileIncreaseCommand() {
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
                                        new RumblePulseCommand(driver,
                                                        0.3,
                                                        0.5,
                                                        RumbleIntensity.SUPER_HEAVY,
                                                        (byte) (currentProfileIndex + 1),
                                                        () -> RumbleType.kBothRumble));
                });
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeProfileDecreaseCommand() {
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
                                        new RumblePulseCommand(driver,
                                                        0.3,
                                                        0.5,
                                                        RumbleIntensity.SUPER_HEAVY,
                                                        (byte) (currentProfileIndex + 1),
                                                        () -> RumbleType.kBothRumble));
                });
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeClimberUpCommand(final Supplier<RumbleType> side) {
                return new ParallelCommandGroup(climber.upward(),
                                RumblePulseCommand.createShortDoublePulse(driver, RumbleIntensity.MEDIUM,
                                                side));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeClimberDownCommand(final Supplier<RumbleType> side) {
                return new ParallelCommandGroup(climber.downward(),
                                new SequentialCommandGroup(
                                                RumblePulseCommand.createShortDoublePulse(driver,
                                                                RumbleIntensity.MEDIUM,
                                                                side),
                                                RumblePulseCommand.createShortWaitCommand(),
                                                RumblePulseCommand.createShortSinglePulse(driver,
                                                                RumbleIntensity.MEDIUM,
                                                                side)));
        }

        // -------------------------------------------------------------------------------------------------------------
        private Command makeAggitorRunCommand(final Supplier<RumbleType> side) {
                return new ParallelCommandGroup(aggitator.run(),
                                RumblePulseCommand.createLongDoublePulse(driver, RumbleIntensity.MEDIUM, side));
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