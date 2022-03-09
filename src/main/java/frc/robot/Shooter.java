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
    private final double HEIGHT = 8 + (2 / 3.);
    private final IRobot robot;
    private double distanceFromHub;
    private double vVector;
    private DigitalInput onDeckSensor;
    private DigitalInput loadedSensor;
    private MotorController launchMotor;
    private MotorController gatekeeperMotor;
    private long startTime;
    private FireState currentState;
    private PIDController shooterPID;
    private Encoder encoder;
    private double previousOutput = 0.0;
    private final double cameraHeight = 1;
    private final double targetHeight = 8 + 8 / 12.;
    private final double cameraAngle = 5 * Math.PI / 36;
    private final double targetAngle = 1;
    private final double SPIN_UP_TIME = 1000;
    private final double EXIT_TIME = 500;

    public Shooter(IRobot robot, double distanceFromHub) {
        this.distanceFromHub = distanceFromHub;
        onDeckSensor = robot.getFirstBallSenor();
        loadedSensor = robot.getSecondBallSensor();
        launchMotor = robot.getShooterMotor();
        gatekeeperMotor = robot.getGateKeepingMotor();
        currentState = FireState.WAIT;
        shooterPID = new PIDController(0.00007, 0, 0.00002);
        encoder = robot.getShooterEncoder();
        //pulses per revolution = 48
        encoder.setDistancePerPulse(Math.PI / 24);
        SmartDashboard.putData("Encoder", encoder);
        SmartDashboard.putData("BallSensor1", onDeckSensor);
        SmartDashboard.putData("BallSensor2", loadedSensor);
        this.robot = robot;
    }

    public void fireUpdate(HIDButton fireButton, boolean onTarget) {
        double gateSpeed = 0;
        double launchSpeed = 0;
        switch (currentState) {
            case WAIT:
                gateSpeed = CHILL_BRO;
                launchSpeed = CHILL_BRO;
                if (fireButton.getValue() && (onDeckSensor.get() || !loadedSensor.get()) && onTarget) {
                    currentState = FireState.SPIN_UP;
                    startTime = System.currentTimeMillis();
                    shooterPID.reset();
                }
                break;
            case SPIN_UP:
                gateSpeed = CHILL_BRO;
                launchSpeed = YOLO; //calculateVVector();
                if (!fireButton.getValue()) {
                    currentState = FireState.WAIT;
                } else if (System.currentTimeMillis() - startTime >= SPIN_UP_TIME) {
                    currentState = FireState.FIRE;
                }
                break;
            case FIRE:
                gateSpeed = YOLO;
                launchSpeed = YOLO; //calculateVVector();
                if (onDeckSensor.get() || loadedSensor.get()) {
                    startTime = System.currentTimeMillis();
                }
                if (!fireButton.getValue() || (System.currentTimeMillis() - startTime >= EXIT_TIME)) {
                    currentState = FireState.WAIT;
                }
                break;
        }
        if (gateSpeed == CHILL_BRO && loadedSensor.get() && !onDeckSensor.get()) {
            gateSpeed = YOLO;
        }
        gatekeeperMotor.set(gateSpeed);
        shooterPID.setSetpoint(launchSpeed);
        launchMotor.set(launchSpeed);
        //updateAndOutputPID();
    }

    //TODO: Get actual x-distance from target using camera
    public double calculateDistance() {
        double currentDistance;
        currentDistance = (targetHeight * cameraHeight) / Math.tan(cameraAngle * targetAngle);
        return currentDistance;
    }

    public double calculateVVector() {
        vVector = Math.sqrt((GRAV / 2) * ((calculateDistance() * calculateDistance()) /
                ((Math.cos(THETA) * Math.cos(THETA)) * (calculateDistance() * Math.tan(THETA) - HEIGHT))));
        return vVector;
    }

    private void updateAndOutputPID() {
        previousOutput += shooterPID.calculate(encoder.getRate());
        launchMotor.set(previousOutput);
        SmartDashboard.putNumber("Launch Motor Val", previousOutput);
    }

    //region Useless Limelight Code we stole off the internet. E
    public void limeLight() {
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
    //endregion
}
