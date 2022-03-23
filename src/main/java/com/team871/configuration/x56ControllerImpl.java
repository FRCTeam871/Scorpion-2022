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

public class x56ControllerImpl implements IController, Sendable {
    private static final double DEFAULT_DEADBAND = .15;

    private final GenericJoystick<X56JoystickButtons, X56JoystickAxes> driveController;
    private final GenericJoystick<XBoxButtons, XBoxAxes> climbController;
    private UpdatedHIDAxis updatedHIDAxis;
    private UpdatedHIDAxis updatedHIDAxis2;

    /**
     * @param drivePort the values of the buttons and axes on the controller
     */
    public x56ControllerImpl(int drivePort, int climbPort) {
        driveController = new GenericJoystick<>(drivePort, Arrays.asList(X56JoystickButtons.values()), Arrays.asList(X56JoystickAxes.values()));
        driveController.getButton(X56JoystickButtons.TRIGGER).setMode(ButtonTypes.MOMENTARY);
        driveController.getButton(X56JoystickButtons.PINKIE_TRIGGER).setMode(ButtonTypes.MOMENTARY);
        driveController.getButton(X56JoystickButtons.D).setMode(ButtonTypes.MOMENTARY);
        driveController.getButton(X56JoystickButtons.TRIGGER).setMode(ButtonTypes.MOMENTARY);

        driveController.getAxis(X56JoystickAxes.X_AXIS).setDeadband(DEFAULT_DEADBAND);
        driveController.getAxis(X56JoystickAxes.Y_AXIS).setDeadband(DEFAULT_DEADBAND);
        driveController.getAxis(X56JoystickAxes.Z_AXIS).setDeadband(DEFAULT_DEADBAND);

        climbController = new GenericJoystick<>(climbPort, Arrays.asList(XBoxButtons.values()), Arrays.asList(XBoxAxes.values()));
        climbController.getButton(XBoxButtons.A).setMode(ButtonTypes.MOMENTARY);
        climbController.getButton(XBoxButtons.B).setMode(ButtonTypes.MOMENTARY);
        climbController.getButton(XBoxButtons.RBUMPER).setMode(ButtonTypes.MOMENTARY);
        climbController.getAxis(XBoxAxes.LEFTY).setDeadband(DEFAULT_DEADBAND);
        climbController.getAxis(XBoxAxes.RIGHTX).setDeadband(DEFAULT_DEADBAND);
        climbController.getAxis(XBoxAxes.RIGHTY).setDeadband(DEFAULT_DEADBAND);
        updatedHIDAxis = new UpdatedHIDAxis(climbController.getAxis(XBoxAxes.RIGHTX), true);
        // updatedHIDAxis2 = new UpdatedHIDAxis(driveController.getAxis(X56JoystickAxes.CSTICK_Y_AXIS), true);

    }

    //region Drive Controller Group
    @Override
    public HIDAxis getDriveX() {
        return driveController.getAxis(X56JoystickAxes.X_AXIS);
    }

    @Override
    public HIDAxis getDriveY() {
        return driveController.getAxis(X56JoystickAxes.Y_AXIS);
    }

    @Override
    public HIDAxis getDriveRotation() {
        return driveController.getAxis(X56JoystickAxes.Z_AXIS);
    }

    @Override
    public HIDButton getRearRightButton() {
        return null;
    }

    @Override
    public HIDButton getRearLeftButton() {
        return null;
    }

    @Override
    public HIDButton getFrontRightButton() {
        return null;
    }

    @Override
    public HIDButton getFrontLeftButton() {
        return null;
    }

    @Override
    public HIDButton getInvertButton() {
        return null;
    }

    @Override
    public HIDButton getFireButton() {
        return driveController.getButton(X56JoystickButtons.PINKIE_TRIGGER);
    }

    @Override
    public HIDAxis getCollectorAxis() {
        return driveController.getAxis(X56JoystickAxes.CSTICK_Y_AXIS);//driveController.getButton(X56JoystickButtons.TRIGGER);
    }

    @Override
    public HIDButton getRegurgitateButton() {
        return driveController.getButton(X56JoystickButtons.D);
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
