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
    private final IRobot robot;
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
    private double previousOutput = 0.0;
    private double cameraHeight = 1;
    private double targetHeight = 8 + 8 / 12;
    private double cameraAngle = 5 * Math.PI / 36;
    private double targetAngle = 1;

    public Shooter(IRobot robot, double distanceFromHub) {
        this.distanceFromHub = distanceFromHub;
        ballSensor1 = robot.getFirstBallSenor();
        ballSensor2 = robot.getSecondBallSensor();
        launchMotor = robot.getShooterMotor();
        gatekeeperMotor = robot.getGateKeepingMotor();
        currentState = FireState.WAIT;
        shooterPID = new PIDController(0.00007, 0, 0.00002);
        encoder = robot.getShooterEncoder();
        //pulses per revolution = 48
        encoder.setDistancePerPulse(Math.PI / 24);
        SmartDashboard.putData("Encoder", encoder);
        this.robot = robot;
    }

    public void fireUpdate(HIDButton fireButton) {
        if (currentState == FireState.WAIT) {
            gatekeeperMotor.set(CHILL_BRO);
            if (fireButton.getValue()) {
                currentState = FireState.SPIN_UP;
                startTime = System.currentTimeMillis();
//                setSetPoint();
            }
//            else {
//                if (fireButton.getValue() && ballSensor1.get() && ballSensor2.get()) {
//                    currentState = FireState.SPIN_UP;
//                    startTime = System.currentTimeMillis();
//                    setSetPoint(600);
//                }
//            }
        }

        if (currentState == FireState.SPIN_UP) {
            launchMotor.set(YOLO);
            if ((System.currentTimeMillis() - startTime) >= 1000) {
                currentState = FireState.FIRE;
            }
        }
        if (currentState == FireState.FIRE) {
            gatekeeperMotor.set(YOLO);
//                if(!ballSensor1.get() && !ballSensor2.get()){
//                    currentState = FireState.WAIT;
//                    setSetPoint(0.);
//                }
            if (!fireButton.getValue()) {
                currentState = FireState.WAIT;
//                    setSetPoint(0.);
                launchMotor.set(0.);
            }

        }
//            else {
//                gatekeeperMotor.set(YOLO);
//                if (!fireButton.getValue()) {
//                    currentState = FireState.WAIT;
//                    setSetPoint(0.);
//                }
//            }

//            updateAndOutputPID();

    }

    public double calculateDistance() {
        double currentDistance;
        currentDistance = (targetHeight * cameraHeight) / Math.tan(cameraAngle * targetAngle);
        return currentDistance;
    }

    public double calculateVVector() {
        vVector = Math.sqrt((1 / 2 * GRAV) * ((calculateDistance() * calculateDistance()) /
                ((Math.cos(THETA) * Math.cos(THETA)) * (calculateDistance() * Math.tan(THETA) - HEIGHT))));
        return vVector;
    }

    public void setSetPoint() {
        shooterPID.setSetpoint(calculateVVector());
        shooterPID.reset();
    }

    public void setSetPoint(double speed) {
        shooterPID.setSetpoint(speed);
        shooterPID.reset();
    }

    private void updateAndOutputPID() {
        previousOutput += shooterPID.calculate(encoder.getRate());
        launchMotor.set(previousOutput);
        SmartDashboard.putNumber("Launch Motor Val", previousOutput);
    }

    public void limeLight() {
//TODO: AHHH

//        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
//        NetworkTableEntry tx = table.getEntry("tx");
//        NetworkTableEntry ty = table.getEntry("ty");
//        NetworkTableEntry ta = table.getEntry("ta");
//
//        //read values periodically
//        double x = tx.getDouble(0.0);
//        double y = ty.getDouble(0.0);
//        double area = ta.getDouble(0.0);
//
//        //post to smart dashboard periodically
//        SmartDashboard.putNumber("LimelightX", x);
//        SmartDashboard.putNumber("LimelightY", y);
//        SmartDashboard.putNumber("LimelightArea", area);
    }
}
