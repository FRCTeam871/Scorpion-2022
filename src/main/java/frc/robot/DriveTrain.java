package frc.robot;

import com.team871.hid.HIDAxis;
import com.team871.hid.HIDButton;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class DriveTrain {
    private MotorController frontLeftMotor;
    private MotorController backRightMotor;
    private MotorController backLeftMotor;
    private MotorController frontRightMotor;
    private PIDController aimPID;
    private PIDController distancePID;
    private MecanumDrive mecanum;
    private Gyro gyro;
    private IRobot config;
    private float Kp = 0.00001f;
    private float minCommand = 0.05f;
    float headingError;
    float steeringAdjust = 0.0f;
    private NetworkTable networkTable;
    private double tx;
    private double leftCommand;
    private double rightCommand;
    //TODO: find real vals

    public DriveTrain(MotorController frontLeftMotor, MotorController frontRightMotor, MotorController backLeftMotor, MotorController backRightMotor) {
        this.frontLeftMotor = frontLeftMotor;
        this.backRightMotor = backRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.frontRightMotor = frontRightMotor;
        mecanum = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
        // TODO: determine actually PID
        aimPID = new PIDController(Kp, 0.000000, 0.000003);
        aimPID.setSetpoint(0.0);
//        distancePID = new PIDController(0.00003, 0.00001,0.00002);
//        distancePID.setSetpoint(16+4/12);
        networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public DriveTrain(IRobot config) {
        gyro = config.getGyro();
    }

    public void driveMecanum(HIDAxis xValue, HIDAxis yValue, HIDAxis zValue, HIDButton aimButton/*,HIDAxis driveY, HIDAxis driveX*/) {
        if (aimButton.getValue()) {
//            if(driveY.getValue()>0||driveX.getValue()>0) {
//                double rotationScale = (Math.abs(.25*yValue.getValue())) + .75;
//                mecanum.driveCartesian(yValue.getValue(), -xValue.getValue(), -zValue.getValue()*rotationScale);
//            } else {
//                mecanum.driveCartesian(yValue.getValue(),xValue.getValue(), aimPID.calculate(tx));
//                //mecanum.driveCartesian(distancePID.calculate(calculateDistance()),xValue.getValue(), aimPID.calculate(tx));
//            }
            //distancePID.calculate(calculateDistance());
            networkTable.getEntry("tx");
            mecanum.driveCartesian(yValue.getValue(), xValue.getValue(), aimPID.calculate(tx));
            //mecanum.driveCartesian(distancePID.calculate(calculateDistance()),xValue.getValue(), aimPID.calculate(tx));
        } else {
            double rotationScale = (Math.abs(.25 * yValue.getValue())) + .75;
            mecanum.driveCartesian(yValue.getValue(), -xValue.getValue(), -zValue.getValue() * rotationScale);
        }
    }

    public void driveTank(double leftValue, double rightValue) {
        frontLeftMotor.set(leftValue);
        backRightMotor.set(rightValue);
        backLeftMotor.set(leftValue);
        frontRightMotor.set(rightValue);
    }

    public void resetGyro() {
        gyro.reset();
    }
}
