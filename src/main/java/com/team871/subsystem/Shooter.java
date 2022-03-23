package com.team871.subsystem;

import com.team871.configuration.IRobot;
import com.team871.hid.HIDButton;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    private static final double YOLO = 1;
    private static final double CHILL_BRO = 0;
    private static final double SPIN_UP_TIME = 1000;
    private static final double EXIT_TIME = 500;
    private static final boolean USE_PID = false;

    //TODO: measure values once robot has been built u f o ol s
//    private final int GRAV = 32;
//    private final double THETA = Math.PI / 3;
//    private final double HEIGHT = 8 + (2 / 3.);
//    private final double cameraHeight = 1;
//    private final double targetHeight = 8 + 8 / 12.;
//    private final double cameraAngle = 5 * Math.PI / 36;
//    private final double targetAngle = 1;

    private final DigitalInput onDeckSensor;
    private final DigitalInput loadedSensor;
    private final MotorController launchMotor;
    private final MotorController gatekeeperMotor;
    private final PIDController shooterPID;
    private final Encoder encoder;

    private final double distanceFromHub;
    private FireState currentState;
    private long startTime;
    private double previousOutput = 0.0;

    private enum FireState {
        WAIT,
        SPIN_UP,
        FIRE,
    }

    public Shooter(IRobot robot, double distanceFromHub) {
        this.distanceFromHub = distanceFromHub;
        onDeckSensor = robot.getOnDeckSensor();
        loadedSensor = robot.getLoadedSensor();
        launchMotor = robot.getShooterMotor();
        gatekeeperMotor = robot.getGateKeepingMotor();
        currentState = FireState.WAIT;

        shooterPID = new PIDController(0.00448, 0, 0.00016);
        encoder = robot.getShooterEncoder();

        // TODO: This seems wrong.  This gives us Radians/tick which translates to Radians/sec for rate.
        //       We really want to compute Rotations/sec because we can easily compute the target value from
        //       the distance to the target.
        //Distance is in revolutions
        encoder.setDistancePerPulse(1. / 48);

        SmartDashboard.putData("Thrower RPS", encoder);
        SmartDashboard.putData("On Deck Sensor", onDeckSensor);
        SmartDashboard.putData("Loaded Sensor", loadedSensor);
        SmartDashboard.putData("Shooter PID", shooterPID);
    }

    public void update(HIDButton fireButton, boolean onTarget) {
      update(fireButton.getValue(), onTarget);
    }

    public void update(boolean fireButton, boolean onTarget) {
        double gateSpeed = 0;
        double launchSpeed = 0;
        switch (currentState) {
            case WAIT:
                gateSpeed = CHILL_BRO;
                launchSpeed = CHILL_BRO;
                if (fireButton && (onDeckSensor.get() || !loadedSensor.get()) && onTarget) {
                    currentState = FireState.SPIN_UP;
                    startTime = System.currentTimeMillis();
                    shooterPID.reset();
                    previousOutput = 0;
                }
                break;

            case SPIN_UP:
                gateSpeed = CHILL_BRO;
                if (!fireButton) {
                    currentState = FireState.WAIT;
                } else if (System.currentTimeMillis() - startTime >= SPIN_UP_TIME) {
                    currentState = FireState.FIRE;
                }
                break;

            case FIRE:
                gateSpeed = YOLO;
                if (!fireButton) {
                    currentState = FireState.WAIT;
                }
                break;
        }

        if (loadedSensor.get() && !onDeckSensor.get()) {
            gateSpeed = YOLO;
        }

        if(currentState == FireState.SPIN_UP || currentState == FireState.FIRE) {
            if(USE_PID) {
                shooterPID.setSetpoint(computeLaunchRPS());
                previousOutput += shooterPID.calculate(encoder.getRate());
                launchSpeed = previousOutput;
            } else {
                launchSpeed = YOLO;
            }
        }

        gatekeeperMotor.set(gateSpeed);
        launchMotor.set(launchSpeed);
        SmartDashboard.putString("Shooter State", currentState.toString());
        SmartDashboard.putNumber("Launch RPS", encoder.getRate());
    }

    //region Compute target RPM
    private double computeLaunchRPS() {
        return 30.;
    }
    //endregion
}
