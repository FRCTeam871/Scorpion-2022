package frc.robot;

import com.team871.hid.ButtonTypes;
import com.team871.hid.GenericJoystick;
import com.team871.hid.HIDAxis;
import com.team871.hid.HIDButton;
import com.team871.hid.joystick.XBoxAxes;
import com.team871.hid.joystick.XBoxButtons;
import java.util.Arrays;

public class XBController implements IController {
    private GenericJoystick<XBoxButtons, XBoxAxes> driveController;
    private GenericJoystick<XBoxButtons, XBoxAxes> climbController;
    private static final double DEADBANDLEFT = .15;
    private static final double DEADBANDRIGHT = .15;

    /**
     * @param drivePort the values of the buttons and axes on the controller
     */
    public XBController(int drivePort, int climbPort) {
        driveController = new GenericJoystick<XBoxButtons, XBoxAxes>(drivePort, Arrays.asList(XBoxButtons.values()), Arrays.asList(XBoxAxes.values()));
        driveController.getButton(XBoxButtons.A).setMode(ButtonTypes.MOMENTARY);
        driveController.getButton(XBoxButtons.B).setMode(ButtonTypes.MOMENTARY);
        driveController.getButton(XBoxButtons.X).setMode(ButtonTypes.MOMENTARY);
        driveController.getButton(XBoxButtons.Y).setMode(ButtonTypes.MOMENTARY);
        driveController.getButton(XBoxButtons.LBUMPER).setMode(ButtonTypes.RISING);
        driveController.getButton(XBoxButtons.RBUMPER).setMode(ButtonTypes.MOMENTARY);
//        driveController.getButton(XBoxButtons.START).setMode(ButtonTypes.RISING);
//        driveController.getButton(XBoxButtons.BACK).setMode(ButtonTypes.RISING);
        driveController.getAxis(XBoxAxes.LEFTX).setDeadband(DEADBANDLEFT);
        driveController.getAxis(XBoxAxes.LEFTY).setDeadband(DEADBANDLEFT);
        driveController.getAxis(XBoxAxes.RIGHTX).setDeadband(DEADBANDRIGHT);

        climbController = new GenericJoystick<XBoxButtons, XBoxAxes>(climbPort, Arrays.asList(XBoxButtons.values()), Arrays.asList(XBoxAxes.values()));
        climbController.getButton(XBoxButtons.A).setMode(ButtonTypes.MOMENTARY);
        climbController.getButton(XBoxButtons.B).setMode(ButtonTypes.MOMENTARY);
        climbController.getAxis(XBoxAxes.LEFTY).setDeadband(DEADBANDLEFT);
        climbController.getAxis(XBoxAxes.RIGHTX).setDeadband(DEADBANDRIGHT);
        climbController.getAxis(XBoxAxes.RIGHTY).setDeadband(DEADBANDRIGHT);
        climbController.getButton(XBoxButtons.RBUMPER).setMode(ButtonTypes.MOMENTARY);
    }

    @Override
    public HIDAxis getDriveX() {
        return driveController.getAxis(XBoxAxes.LEFTX);
    }

    @Override
    public HIDAxis getDriveY() {
        return driveController.getAxis(XBoxAxes.LEFTY);
    }

    @Override
    public HIDAxis getDriveZ() {
        return driveController.getAxis(XBoxAxes.RIGHTX);
    }

    @Override
    public HIDAxis getLeftAxis() {
        return driveController.getAxis(XBoxAxes.LEFTY);
    }

    @Override
    public HIDAxis getRightAxis() {
        return driveController.getAxis(XBoxAxes.LEFTX);
    }

    @Override
    public HIDAxis getClimbGrabAxis() {
        return climbController.getAxis(XBoxAxes.LEFTY);
    }

    @Override
    public HIDAxis getClimbSwingAxis() {
        return climbController.getAxis(XBoxAxes.RIGHTX);
    }

    @Override
    public HIDAxis getAltClimbSwingAxis() {
        return null;
    }

    @Override
    public HIDButton getActivateSwingPIDButton() {
        return climbController.getButton(XBoxButtons.RBUMPER);
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

    @Override
    public HIDButton getClimbButton() {
        return climbController.getButton(XBoxButtons.A);
    }

    @Override
    public HIDButton getRevertClimbButton() {
        return climbController.getButton(XBoxButtons.B);
    }
}

