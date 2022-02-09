package frc.robot;

import com.team871.hid.HIDButton;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    private static final double YOLO = 1;
    public static final int CHILL_BRO = 0;
    //TODO: measure values once robot has been built u f o ol s
    private final int GRAV = 32;
    private final double THETA = Math.PI / 3;
    private final double HEIGHT = 8 + (2 / 3);
    private double distanceFromHub;
    private double vVector;
    private DigitalInput ballSensor1;
    private DigitalInput ballSensor2;
    private MotorController launchMotor;
    private MotorController gatekeeperMotor;
    private long startTime;
    private FireState currentState;
    private PIDController shooterPID;
    private Encoder encoder;
    private Double previousRPM;

    public Shooter(IRobot robot, double distanceFromHub) {
        this.distanceFromHub = distanceFromHub;
        ballSensor1 = robot.getFirstBallSenor();
        ballSensor2 = robot.getSecondBallSensor();
        launchMotor = robot.getShooterMotor();
        gatekeeperMotor = robot.getGateKeepingMotor();
        currentState = FireState.WAIT;
        shooterPID = new PIDController(0.01, 0, 0.001);
        encoder = robot.getShooterEncoder();
        //pulses per revolution = 48
        encoder.setDistancePerPulse(Math.PI / 24);
        previousRPM = 0.0;
        SmartDashboard.putData("Encoder", encoder);
    }

    public void fireUpdate(HIDButton fireButton) {
        if (currentState == FireState.WAIT) {
            launchMotor.set(CHILL_BRO);
            gatekeeperMotor.set(CHILL_BRO);

            if (fireButton.getValue()) {
                currentState = FireState.SPIN_UP;
                startTime = System.currentTimeMillis();
            }
        }
        if (currentState == FireState.SPIN_UP) {
            launchMotor.set(YOLO);
            if ((System.currentTimeMillis() - startTime) >= 1000) {
                currentState = FireState.FIRE;
            }
        }

        if (currentState == FireState.FIRE) {
            gatekeeperMotor.set(YOLO);
            if (!fireButton.getValue()) {
                currentState = FireState.WAIT;
            }
        }
    }

    public double calculateVVector() {
        vVector = Math.sqrt((1 / 2 * GRAV) * ((distanceFromHub * distanceFromHub) /
                ((Math.cos(THETA) * Math.cos(THETA)) * (distanceFromHub * Math.tan(THETA) - HEIGHT))));
        return vVector;
    }
    public void setRPM(double setPoint){
        gatekeeperMotor.set(YOLO);
        launchMotor.set(previousRPM + (shooterPID.calculate(encoder.getRate(), setPoint) / 350));
        previousRPM = encoder.getRate();
    }
}
