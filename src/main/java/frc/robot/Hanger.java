package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.team871.hid.HIDAxis;
import com.team871.hid.HIDButton;
import com.team871.hid.joystick.XBoxButtons;
import com.team871.io.sensor.DigitalLimitSwitch;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hanger {

    private MotorController leftGrabMotor;
    private MotorController rightGrabMotor;
    private MotorController leftSwingMotor;
    private MotorController rightSwingMotor;
    private DigitalLimitSwitch grabHookedLimitSwitch;
    private DigitalLimitSwitch swingHookedLimitSwitch;
    private DigitalLimitSwitch swingForwardLimitSwitch;
    private DigitalLimitSwitch swingBackLimitSwitch;
    private DigitalLimitSwitch fullExtendLimitSwitch;
    private DigitalLimitSwitch fullRetractLimitSwtich;
    private ClimbState state;
    private final double GRAB_SPEED = .5;
    private final double SWING_SPEED = .5;
    private final double CHILL_BRO = 0.0;
    private PIDController pitchPID;
    private double previousAngle = 0;
    private AHRS gyro;
    private double position;

    public Hanger(IRobot robot) {
        leftGrabMotor = robot.getLeftGrabArm();
        rightGrabMotor = robot.getRightGrabArm();
        leftSwingMotor = robot.getSwingArmLeft();
        rightSwingMotor = robot.getSwingArmRight();
        //TODO: Uncomment when the robot has these limit switches
//        grabHookedLimitSwitch = robot.getGrabHookedLimitSwitch();
//        swingHookedLimitSwitch = robot.getSwingHookedLimitSwtich();
//        fullExtendLimitSwitch = robot.getFullExtendLimitSwitch();
        fullRetractLimitSwtich = robot.getFullRetractLimitSwitch();
        state = ClimbState.FORWARD_RETRACT;
        swingForwardLimitSwitch = robot.getSwingForwardLimitSwitch();
        swingBackLimitSwitch = robot.getSwingBackLimitSwitch();
        grabHookedLimitSwitch = robot.getGrabHookedLimitSwitch();
        pitchPID = new PIDController(0.01111, 0., 0.00555);
        gyro = robot.getGyro();
        //TODO: Uncomment
//        SmartDashboard.putData("Grab Extended", fullExtendLimitSwitch.getRawInput());
        SmartDashboard.putData("Grab Retracted", fullRetractLimitSwtich.getRawInput());
        SmartDashboard.putData("Swing Forward", swingForwardLimitSwitch.getRawInput());
        SmartDashboard.putData("Swing Backwards", swingBackLimitSwitch.getRawInput());
        SmartDashboard.putData("PitchPID", pitchPID);

    }

    private void extendArms() {
        if (fullExtendLimitSwitch.get()) {
            leftGrabMotor.set(CHILL_BRO);
            rightGrabMotor.set(CHILL_BRO);
        } else {
            leftGrabMotor.set(GRAB_SPEED);
            rightGrabMotor.set(GRAB_SPEED);
        }
    }

    private void retractArms() {
        if (!fullRetractLimitSwtich.get()) {
            leftGrabMotor.set(CHILL_BRO);
            rightGrabMotor.set(CHILL_BRO);
        } else {
            leftGrabMotor.set(-GRAB_SPEED);
            rightGrabMotor.set(-GRAB_SPEED);
        }
    }

    private void swingForward() {
        if (swingForwardLimitSwitch.get()) {
            rightSwingMotor.set(CHILL_BRO);
            leftSwingMotor.set(CHILL_BRO);
        } else {
            rightSwingMotor.set(SWING_SPEED);
            leftSwingMotor.set(SWING_SPEED);
        }
    }

    private void swingBackwards() {
        if (!swingBackLimitSwitch.get()) {
            rightSwingMotor.set(CHILL_BRO);
            leftSwingMotor.set(CHILL_BRO);
        } else {
            rightSwingMotor.set(-SWING_SPEED);
            leftSwingMotor.set(-SWING_SPEED);
        }
    }

    public void dumbUpdate() {

    }


    //TODO: finish state machine web
    public void update(HIDButton climbRequested, HIDButton emergencyBack) {
        switch (state) {
            case FORWARD_RETRACT:
                retractArms();
                swingForward();
                if (climbRequested.getValue()) {
                    state = ClimbState.BACKWARDS_EXTEND;
                }
                break;
            case FORWARD_EXTEND:
                extendArms();
                swingForward();
                break;
            case BACKWARDS_RETRACT:
                retractArms();
                swingBackwards();
                break;
            case BACKWARDS_EXTEND:
                extendArms();
                swingBackwards();
                if (climbRequested.getValue()) {
                    state = ClimbState.BACKWARDS_RETRACT;
                } else if (emergencyBack.getValue()) {
                    state = ClimbState.FORWARD_RETRACT;
                }
                break;
        }


//        switch (state) {
//            case STARTING:
//                if (swingForwardLimitSwitch.get()) {
//                    leftSwingMotor.set(CHILL_BRO);
//                    rightSwingMotor.set(CHILL_BRO);
//                } else {
//                    leftSwingMotor.set(SWING_SPEED);
//                    rightSwingMotor.set(SWING_SPEED);
//                }
//                if (fullRetractLimitSwtich.get()) {
//                    leftGrabMotor.set(CHILL_BRO);
//                    rightGrabMotor.set(CHILL_BRO);
//                } else {
//                    leftGrabMotor.set(-GRAB_SPEED);
//                    rightGrabMotor.set(-GRAB_SPEED);
//                }
//                if (climbRequested.getRaw()) {
//                    state = ClimbState.EXTEND;
//                }
//                    break;
//
//            case EXTEND:
//                if (fullExtendLimitSwitch.get()) {
//                    leftGrabMotor.set(CHILL_BRO);
//                    rightGrabMotor.set(CHILL_BRO);
//                } else {
//                    leftGrabMotor.set(GRAB_SPEED);
//                    rightGrabMotor.set(GRAB_SPEED);
//                }
//                if (swingBackLimitSwitch.get()) {
//                    leftSwingMotor.set(CHILL_BRO);
//                    rightSwingMotor.set(CHILL_BRO);
//                } else {
//                    leftSwingMotor.set(-SWING_SPEED);
//                    rightSwingMotor.set(-SWING_SPEED);
//                }
//                if(climbRequested.getValue()) {
//                    state = ClimbState.PULL_UP;
//                    //TODO: make sure that this works(aka: you only need to set it once and the speed stays until changed)
////                    leftGrabMotor.set(-GRAB_SPEED);
////                    rightGrabMotor.set(-GRAB_SPEED);
////                    leftSwingMotor.set(CHILL_BRO);
////                    rightSwingMotor.set(CHILL_BRO);
//                }
//                if(emergencyBack.getValue()){
//                    state = ClimbState.STARTING;
//                }
//                    break;
//
//            case PULL_UP:
// //                 if()
//                if(fullRetractLimitSwtich.get()) {
//                    state = ClimbState.SWING_OVER;
//                    leftGrabMotor.set(0);
//                    rightGrabMotor.set(0);
////                    swingArmsMotor.set(-.5);
//                } else {
//                    leftGrabMotor.set(-.5);
//                    rightGrabMotor.set(-.5);
//                    }
//                if(emergencyBack.getValue()){
//                    state = ClimbState.EXTEND;
//                }
//                    break;
//
//             case SWING_OVER:
//                 if (swingForwardLimitSwitch.get()) {
//                     state = ClimbState.HANDOFF;
//                     leftSwingMotor.set(CHILL_BRO);
//                     rightSwingMotor.set(CHILL_BRO);
//                 } else {
//                     leftSwingMotor.set(SWING_SPEED);
//                     rightSwingMotor.set(SWING_SPEED);
//                 } if(emergencyBack.getValue()){
//                 state = ClimbState.PULL_UP;
//             }
//                    break;
//
//            case HANDOFF:
//                if (swingHookedLimitSwitch.get()) {
//                    state = ClimbState.EXTEND;
//                    leftGrabMotor.set(CHILL_BRO);
//                    rightGrabMotor.set(CHILL_BRO);
//                } else {
//                    leftGrabMotor.set(GRAB_SPEED);
//                    rightGrabMotor.set(GRAB_SPEED);
////                    leftSwingMotor.set(SWING_SPEED);
////                    rightSwingMotor.set(SWING_SPEED);
//                }
//                if(emergencyBack.getValue()){
//                    state = ClimbState.SWING_OVER;
//                }
//                    break;
//        }
    }

    public void dirveClimber(HIDAxis grabAxis, HIDAxis swingAxis) {
        double factor = .25;
        double grabSpeed = grabAxis.getValue();
        double swingSpeed = swingAxis.getValue();
        //TODO: Uncomment when limit switch is on robor
        if ((grabSpeed < 0 /*&& !fullExtendLimitSwitch.get()*/) || (grabSpeed > 0 && fullRetractLimitSwtich.get())) {
            leftGrabMotor.set(grabSpeed);
            rightGrabMotor.set(grabSpeed);
        } else {
            leftGrabMotor.set(0);
            rightGrabMotor.set(0);
        }
        if ((swingSpeed > 0 && !swingForwardLimitSwitch.get()) || (swingSpeed < 0 && swingBackLimitSwitch.get())) {
            leftSwingMotor.set((swingSpeed) * ((swingAxis.getValue() < 0) ? .1 : 1));
            rightSwingMotor.set((swingSpeed) * ((swingAxis.getValue() < 0) ? .1 : 1));
        } else {
            leftSwingMotor.set(0);
            rightSwingMotor.set(0);
        }
        SmartDashboard.putNumber("swingSpeed", (swingSpeed) * ((swingAxis.getValue() < 0) ? .1 : 1));
        SmartDashboard.putNumber("grabSpeed", grabSpeed);
        SmartDashboard.putNumber("Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Yaw", gyro.getYaw());
        SmartDashboard.putNumber("Roll", gyro.getRoll());
    }

    //TODO: Fix so that user can set the setpoint ~ Gamer
    public void calcatePitchPID(HIDAxis swingAxis) {
        double targetAngle = (swingAxis.getValue() + 1) * 22.5;
        final double tiltSpeed = pitchPID.calculate(Math.abs(gyro.getRoll()), targetAngle);
        SmartDashboard.putNumber("tiltSpeed", tiltSpeed);
        SmartDashboard.putNumber("targetAngle", targetAngle);
        SmartDashboard.putNumber("Roll", gyro.getRoll());
        leftSwingMotor.set(-tiltSpeed);
        rightSwingMotor.set(-tiltSpeed);
    }

}