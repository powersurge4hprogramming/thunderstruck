// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;

public class Robot extends TimedRobot {
    private final RobotSystem robotContainer;

    // Grabbed from this.autonomousInit().
    private Command autonomousCommand;

    public Robot() {
        robotContainer = new RobotSystem();
        UsbCamera rearCamera = CameraServer.startAutomaticCapture();
        rearCamera.setResolution(320, 320);
        rearCamera.setFPS(15);
    }

    @Override
    public void robotPeriodic() {
        this.robotContainer.updatePhotonCameraFrames();
        this.robotContainer.getCommandScheduler().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            this.robotContainer.getCommandScheduler().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            this.robotContainer.getCommandScheduler().cancel(autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        this.robotContainer.getCommandScheduler().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
