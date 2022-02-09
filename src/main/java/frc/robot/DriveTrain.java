package frc.robot;

import com.team871.hid.HIDAxis;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class DriveTrain {
    private MotorController frontLeftMotor;
    private MotorController backRightMotor;
    private MotorController backLeftMotor;
    private MotorController frontRightMotor;
    private MecanumDrive mecanum;
    private Gyro gyro;
    private IRobot config;

    public DriveTrain(MotorController frontLeftMotor, MotorController frontRightMotor, MotorController backLeftMotor, MotorController backRightMotor) {
        this.frontLeftMotor = frontLeftMotor;
        this.backRightMotor = backRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.frontRightMotor = frontRightMotor;
        mecanum = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
    }
    public DriveTrain(IRobot config) {
        gyro = config.getGyro();
    }
    public void driveMecanum(HIDAxis xValue, HIDAxis yValue, HIDAxis zValue) {
        double rotationScale = (Math.abs(.25*yValue.getValue())) + .75;
        mecanum.driveCartesian(yValue.getValue(), -xValue.getValue(), -zValue.getValue()*rotationScale);
    }
    public void driveTank(double leftValue, double rightValue) {
        frontLeftMotor.set(leftValue);
        backRightMotor.set(rightValue);
        backLeftMotor.set(leftValue);
        frontRightMotor.set(rightValue);
    }
    public void resetGyro(){
        gyro.reset();
    }
}
