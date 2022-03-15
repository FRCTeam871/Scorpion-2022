package com.team871.subsystem;

import com.kauailabs.navx.frc.AHRS;
import com.team871.configuration.IRobot;
import com.team871.hid.HIDAxis;
import com.team871.hid.HIDButton;
import com.team871.io.sensor.DigitalLimitSwitch;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hanger {
    private final MotorController leftGrabMotor;
    private final MotorController rightGrabMotor;
    private final MotorController leftSwingMotor;
    private final MotorController rightSwingMotor;
    private final DigitalLimitSwitch swingForwardLimitSwitch;
    private final DigitalLimitSwitch swingBackLimitSwitch;
    private final DigitalLimitSwitch fullExtendLimitSwitch;
    private final DigitalLimitSwitch fullRetractLimitSwitch;

    private final PIDController pitchPID;
    private final AHRS gyro;

//    private final double GRAB_SPEED = .5;
//    private final double SWING_SPEED = .5;
//    private final double CHILL_BRO = 0.0;
//    private final ClimbState state;

    public Hanger(IRobot robot) {
        leftGrabMotor = robot.getLeftGrabArm();
        rightGrabMotor = robot.getRightGrabArm();
        leftSwingMotor = robot.getSwingArmLeft();
        rightSwingMotor = robot.getSwingArmRight();

        fullExtendLimitSwitch = robot.getFullExtendLimitSwitch();
        fullRetractLimitSwitch = robot.getFullRetractLimitSwitch();
        swingForwardLimitSwitch = robot.getSwingForwardLimitSwitch();
        swingBackLimitSwitch = robot.getSwingBackLimitSwitch();

        pitchPID = new PIDController(0.02, 0., 0.006);
        gyro = robot.getGyro();

        //state = ClimbState.FORWARD_RETRACT;

        SmartDashboard.putData("Grab Extended", fullExtendLimitSwitch.getRawInput());
        SmartDashboard.putData("Grab Retracted", fullRetractLimitSwitch.getRawInput());
        SmartDashboard.putData("Swing Forward", swingForwardLimitSwitch.getRawInput());
        SmartDashboard.putData("Swing Backwards", swingBackLimitSwitch.getRawInput());
        SmartDashboard.putData("PitchPID", pitchPID);
    }

    public void update(HIDAxis swingAxis, HIDAxis grabAxis, HIDButton pidButton) {
        double grabSpeed = grabAxis.getValue();
        double swingSpeed;

        if (pidButton.getValue()) {
            final double targetAngle = (swingAxis.getValue() + 1) * 22.5;
            swingSpeed = -pitchPID.calculate(Math.abs(gyro.getRoll()), targetAngle);
        } else {
            swingSpeed = swingAxis.getValue() * .2;
        }

        if ((grabSpeed < 0 && !fullExtendLimitSwitch.get()) || (grabSpeed > 0 && !fullRetractLimitSwitch.get())) {
            grabSpeed = 0;
        }

        if ((swingSpeed > 0 && !swingForwardLimitSwitch.get()) || (swingSpeed < 0 && !swingBackLimitSwitch.get())) {
            swingSpeed = 0;
        }

        leftGrabMotor.set(grabSpeed);
        rightGrabMotor.set(grabSpeed);
        leftSwingMotor.set(swingSpeed);
        rightSwingMotor.set(swingSpeed);

        SmartDashboard.putNumber("Robot Pitch", gyro.getRoll());
    }

    //region GO AWAY DONT LOOK AT ME
//    private void extendArms() {
//        if (fullExtendLimitSwitch.get()) {
//            leftGrabMotor.set(CHILL_BRO);
//            rightGrabMotor.set(CHILL_BRO);
//        } else {
//            leftGrabMotor.set(GRAB_SPEED);
//            rightGrabMotor.set(GRAB_SPEED);
//        }
//    }
//
//    private void retractArms() {
//        if (!fullRetractLimitSwtich.get()) {
//            leftGrabMotor.set(CHILL_BRO);
//            rightGrabMotor.set(CHILL_BRO);
//        } else {
//            leftGrabMotor.set(-GRAB_SPEED);
//            rightGrabMotor.set(-GRAB_SPEED);
//        }
//    }
//
//    private void swingForward() {
//        if (swingForwardLimitSwitch.get()) {
//            rightSwingMotor.set(CHILL_BRO);
//            leftSwingMotor.set(CHILL_BRO);
//        } else {
//            rightSwingMotor.set(SWING_SPEED);
//            leftSwingMotor.set(SWING_SPEED);
//        }
//    }
//
//    private void swingBackwards() {
//        if (!swingBackLimitSwitch.get()) {
//            rightSwingMotor.set(CHILL_BRO);
//            leftSwingMotor.set(CHILL_BRO);
//        } else {
//            rightSwingMotor.set(-SWING_SPEED);
//            leftSwingMotor.set(-SWING_SPEED);
//        }
//    }
    //endregion
}