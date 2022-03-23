package com.team871.subsystem;

import com.team871.hid.HIDAxis;
import com.team871.hid.HIDButton;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain {
    private final NetworkTable limeLightTable;
    private final PIDController aimPID;
    private final MecanumDrive mecanum;

    public DriveTrain(MotorController frontLeftMotor,
                      MotorController frontRightMotor,
                      MotorController backLeftMotor,
                      MotorController backRightMotor) {
        mecanum = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
        aimPID = new PIDController(0.03, 0.0, 0.00001);
        aimPID.setSetpoint(0.0);
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
        SmartDashboard.putData("AimPID", aimPID);
    }

    public void driveMecanum(HIDAxis xValue, HIDAxis yValue, HIDAxis zValue, HIDButton aimButton) {
        driveMecanum(xValue.getValue(), yValue.getValue(), zValue.getValue(), aimButton.getValue());
    }

    public void driveMecanum(double xValue, double yValue, double zValue, boolean aimButton){
        double rotationScale = (Math.abs(.25 * yValue)) + .75;
        if (aimButton) {
            mecanum.driveCartesian(yValue, -xValue, calculateAim());
            limeLightTable.getEntry("ledMode").setNumber(0);
        } else {
            mecanum.driveCartesian(yValue, -xValue, -zValue * rotationScale);
            limeLightTable.getEntry("ledMode").setNumber(1);
        }
    }

    public double calculateAim() {
        return aimPID.calculate(limeLightTable.getEntry("tx").getValue().getDouble());
    }

    public boolean isOnTarget() {
        return aimPID.atSetpoint();
    }

}
