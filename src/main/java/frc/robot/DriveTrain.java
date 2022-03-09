package frc.robot;

import com.team871.hid.HIDAxis;
import com.team871.hid.HIDButton;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain {
    private MotorController frontLeftMotor;
    private MotorController backRightMotor;
    private MotorController backLeftMotor;
    private MotorController frontRightMotor;
    private PIDController aimPID;
    private MecanumDrive mecanum;
    private Gyro gyro;
    private float minCommand = 0.05f;
    float headingError;
    float steeringAdjust = 0.0f;
    private NetworkTable networkTable;
    private double previousOutput;

    public DriveTrain(MotorController frontLeftMotor, MotorController frontRightMotor, MotorController backLeftMotor, MotorController backRightMotor) {
        this.frontLeftMotor = frontLeftMotor;
        this.backRightMotor = backRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.frontRightMotor = frontRightMotor;
        mecanum = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
        // TODO: determine actually PID
        aimPID = new PIDController(0.03, 0.0, 0.00001);
        aimPID.setSetpoint(0.0);
        networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        SmartDashboard.putData("AimPIDC", aimPID);
    }

    public DriveTrain(IRobot config) {
        gyro = config.getGyro();
    }

    public void driveMecanum(HIDAxis xValue, HIDAxis yValue, HIDAxis zValue, HIDButton aimButton) {
        double rotationScale = (Math.abs(.25 * yValue.getValue())) + .75;
        if (aimButton.getValue()) {
            mecanum.driveCartesian(yValue.getValue(), -xValue.getValue(), calculateAim());
            networkTable.getEntry("ledMode").setNumber(0);
        } else {
            mecanum.driveCartesian(yValue.getValue(), -xValue.getValue(), -zValue.getValue() * rotationScale);
            networkTable.getEntry("ledMode").setNumber(1);
        }
    }

    public double calculateAim() {
        double aimPIDValue = aimPID.calculate(networkTable.getEntry("tx").getValue().getDouble());
        SmartDashboard.putNumber("aimPIDValue", aimPIDValue);
        return aimPIDValue;
    }

    public boolean isOnTarget() {
        return aimPID.atSetpoint();
    }

    public void resetGyro() {
        gyro.reset();
    }
}
