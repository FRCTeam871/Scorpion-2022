package com.team871.configuration;

import com.team871.hid.ButtonTypes;
import com.team871.hid.GenericJoystick;
import com.team871.hid.HIDAxis;
import com.team871.hid.HIDButton;
import com.team871.hid.UpdatedHIDAxis;
import com.team871.hid.joystick.XBoxAxes;
import com.team871.hid.joystick.XBoxButtons;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import java.util.Arrays;

public class XBoxControlImpl implements IController, Sendable {
    private static final double DEFAULT_DEADBAND = .15;

    private final GenericJoystick<XBoxButtons, XBoxAxes> driveController;
    private final GenericJoystick<XBoxButtons, XBoxAxes> climbController;
    private UpdatedHIDAxis updatedHIDAxis;
    /**
     * @param drivePort the values of the buttons and axes on the controller
     */
    public XBoxControlImpl(int drivePort, int climbPort) {
        driveController = new GenericJoystick<>(drivePort, Arrays.asList(XBoxButtons.values()), Arrays.asList(XBoxAxes.values()));
        driveController.getButton(XBoxButtons.A).setMode(ButtonTypes.MOMENTARY);
        driveController.getButton(XBoxButtons.B).setMode(ButtonTypes.MOMENTARY);
        driveController.getButton(XBoxButtons.X).setMode(ButtonTypes.MOMENTARY);
        driveController.getButton(XBoxButtons.Y).setMode(ButtonTypes.MOMENTARY);
        driveController.getButton(XBoxButtons.LBUMPER).setMode(ButtonTypes.RISING);
        driveController.getButton(XBoxButtons.RBUMPER).setMode(ButtonTypes.MOMENTARY);
        driveController.getAxis(XBoxAxes.LEFTX).setDeadband(DEFAULT_DEADBAND);
        driveController.getAxis(XBoxAxes.LEFTY).setDeadband(DEFAULT_DEADBAND);
        driveController.getAxis(XBoxAxes.RIGHTX).setDeadband(DEFAULT_DEADBAND);

        climbController = new GenericJoystick<>(climbPort, Arrays.asList(XBoxButtons.values()), Arrays.asList(XBoxAxes.values()));
        climbController.getButton(XBoxButtons.A).setMode(ButtonTypes.MOMENTARY);
        climbController.getButton(XBoxButtons.B).setMode(ButtonTypes.MOMENTARY);
        climbController.getButton(XBoxButtons.RBUMPER).setMode(ButtonTypes.MOMENTARY);
        climbController.getAxis(XBoxAxes.LEFTY).setDeadband(DEFAULT_DEADBAND);
        climbController.getAxis(XBoxAxes.RIGHTX).setDeadband(DEFAULT_DEADBAND);
        climbController.getAxis(XBoxAxes.RIGHTY).setDeadband(DEFAULT_DEADBAND);

        updatedHIDAxis = new UpdatedHIDAxis(climbController.getAxis(XBoxAxes.RIGHTX), true);
    }

    //region Drive Controller Group
    @Override
    public HIDAxis getDriveX() {
        return driveController.getAxis(XBoxAxes.LEFTX);
    }

    @Override
    public HIDAxis getDriveY() {
        return driveController.getAxis(XBoxAxes.LEFTY);
    }

    @Override
    public HIDAxis getDriveRotation() {
        return driveController.getAxis(XBoxAxes.RIGHTX);
    }

    @Override
    public HIDButton getRearRightButton() {
        return driveController.getButton(XBoxButtons.B);
    }

    @Override
    public HIDButton getRearLeftButton() {
        return driveController.getButton(XBoxButtons.A);
    }

    @Override
    public HIDButton getFrontRightButton() {
        return driveController.getButton(XBoxButtons.Y);
    }

    @Override
    public HIDButton getFrontLeftButton() {
        return driveController.getButton(XBoxButtons.X);
    }

    @Override
    public HIDButton getInvertButton() {
        return driveController.getButton(XBoxButtons.LBUMPER);
    }

    @Override
    public HIDButton getFireButton() {
        return driveController.getButton(XBoxButtons.Y);
    }

    @Override
    public HIDAxis getCollectorAxis() {
        return driveController.getAxis(XBoxAxes.RTRIGGER);
    }

    @Override
    public HIDButton getRegurgitateButton() {
        return driveController.getButton(XBoxButtons.X);
    }
    //endregion

    //region Systems Control Group
    @Override
    public HIDAxis getClimbGrabAxis() {
        return climbController.getAxis(XBoxAxes.LEFTY);
    }

    @Override
    public HIDAxis getClimbSwingAxis() {
        return updatedHIDAxis;
    }

    @Override
    public HIDButton getActivateSwingPIDButton() {
        return climbController.getButton(XBoxButtons.RBUMPER);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Drive X Deadband", () -> getDriveX().getDeadband(), db -> getDriveX().setDeadband(db));
        builder.addDoubleProperty("Drive Y Deadband", () -> getDriveY().getDeadband(), db -> getDriveY().setDeadband(db));
        builder.addDoubleProperty("Drive Rot Deadband", () -> getDriveRotation().getDeadband(), db -> getDriveRotation().setDeadband(db));

        builder.addDoubleProperty("Lift UpDown Deadband", () -> getClimbGrabAxis().getDeadband(), db -> getClimbGrabAxis().setDeadband(db));
        builder.addDoubleProperty("Lift Swing Deadband", () -> getClimbSwingAxis().getDeadband(), db -> getClimbSwingAxis().setDeadband(db));
    }
    //endregion
}
