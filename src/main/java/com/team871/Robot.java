// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team871;

import com.team871.configuration.IController;
import com.team871.configuration.IRobot;
import com.team871.configuration.RobotConfig;
import com.team871.configuration.XBoxControlImpl;
import com.team871.configuration.x56ControllerImpl;
import com.team871.subsystem.Collector;
import com.team871.subsystem.DriveTrain;
import com.team871.subsystem.Hanger;
import com.team871.subsystem.Shooter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private IController xbox;
    private DriveTrain drive;
    private IRobot config;
    private Collector collector;
    private Shooter shooter;
    private Hanger hanger;

    //region autonMode
    private long currentTime = 0;
    private Auton currentState;
    currentState =Auton.START;

    public enum Auton {
        START,
        DRIVE,
        FIRE,
        POWEROFF,
    }

    @Override
    public void autonomousInit() {
        switch (currentState)
        case START:
        currentTime = System.currentTimeMillis();

    }

    @Override
    public void autonomousPeriodic() {

    }
    //regionend

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        xbox = new XBoxControlImpl(0, 1);
        // xbox = new x56ControllerImpl(0, 1);
        // SmartDashboard.putData("ControlImpl", xbox);

        config = new RobotConfig();

        drive = new DriveTrain(config.getFrontLeftMotor(), config.getFrontRightMotor(), config.getRearLeftMotor(), config.getRearRightMotor());
        collector = new Collector(config);
        shooter = new Shooter(config, 6);
        hanger = new Hanger(config);
    }
    
    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        drive.driveMecanum(xbox.getDriveX(), xbox.getDriveY(), xbox.getDriveRotation(), xbox.getFireButton());
        shooter.update(xbox.getFireButton(), drive.isOnTarget());
        collector.activateCollector(xbox.getCollectorAxis());
        // collector.activateCollectorButton(xbox.getCollectorAxis());
        collector.invertCollector(xbox.getRegurgitateButton());
        hanger.update(xbox.getClimbSwingAxis(), xbox.getClimbGrabAxis(), xbox.getActivateSwingPIDButton());
    }
}
