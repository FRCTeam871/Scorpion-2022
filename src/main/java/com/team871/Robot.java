// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team871;

import com.team871.configuration.IController;
import com.team871.configuration.IRobot;
import com.team871.configuration.RobotConfig;
import com.team871.configuration.XBoxControlImpl;
import com.team871.subsystem.Collector;
import com.team871.subsystem.DriveTrain;
import com.team871.subsystem.Hanger;
import com.team871.subsystem.Shooter;
import edu.wpi.first.wpilibj.TimedRobot;

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
    private long autonTimer = 0;
    private AutonState currentState = AutonState.START;

    public enum AutonState {
        START,
        DRIVE,
        FIRE,
        POWEROFF,
    }

    @Override
    public void autonomousInit() {
        currentState = AutonState.START;
    }

    @Override
    public void autonomousPeriodic() {
        int driveSpeed = 0;
        double xValue = 0;
        double yValue = 0;
        double zValue = 0;
        boolean aimButton = false;
        boolean fire = false;

        switch (currentState) {
            case START:
                autonTimer = System.currentTimeMillis();
                currentState = AutonState.DRIVE;
                break;

            case DRIVE:
                yValue = -0.5;
                if(System.currentTimeMillis() - autonTimer >= 2000) {
                    currentState = AutonState.FIRE;
                    autonTimer = System.currentTimeMillis();
                }
                break;

            case FIRE:
                yValue = 0;
                fire = true;
                if(System.currentTimeMillis() - autonTimer >= 2000){
                    currentState = AutonState.POWEROFF;
                    // TODO: GO into FIND_BALLY
                }
                break;

            /*
            case FIND_BALLY:
               // find it
               break;

               case GET_BALLY:
               // get it
               break

            case FIND_TARGETY:
               // Find it too
               // then go shooty
               break;

             */

            case POWEROFF:
                break;
        }

        drive.driveMecanum(xValue, yValue, zValue, aimButton);
        shooter.update(fire, drive.isOnTarget());
    }
    //endregion

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
        collector.invertCollector(xbox.getRegurgitateButton());
        hanger.update(xbox.getClimbSwingAxis(), xbox.getClimbGrabAxis(), xbox.getActivateSwingPIDButton());
    }
}
