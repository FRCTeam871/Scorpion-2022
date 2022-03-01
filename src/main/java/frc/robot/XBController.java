package frc.robot;

import com.team871.hid.ButtonTypes;
import com.team871.hid.GenericJoystick;
import com.team871.hid.HIDAxis;
import com.team871.hid.HIDButton;
import com.team871.hid.joystick.XBoxAxes;
import com.team871.hid.joystick.XBoxButtons;
import java.util.Arrays;

public class XBController implements IController {
    private GenericJoystick<XBoxButtons, XBoxAxes> xboxController;
    private static final double DEADBANDLEFT = .15;
    private static final double DEADBANDRIGHT = .15;
    /**
     * @param prit the values of the buttons and axes on the controller
     */
    public XBController(int prit) {
        xboxController = new GenericJoystick<XBoxButtons, XBoxAxes>(prit, Arrays.asList(XBoxButtons.values()), Arrays.asList(XBoxAxes.values()));
        xboxController.getButton(XBoxButtons.LBUMPER).setMode(ButtonTypes.RISING);
        xboxController.getButton(XBoxButtons.A).setMode(ButtonTypes.MOMENTARY);
        xboxController.getButton(XBoxButtons.B).setMode(ButtonTypes.MOMENTARY);
        xboxController.getButton(XBoxButtons.X).setMode(ButtonTypes.MOMENTARY);
        xboxController.getButton(XBoxButtons.Y).setMode(ButtonTypes.MOMENTARY);
        xboxController.getAxis(XBoxAxes.LEFTX).setDeadband(DEADBANDLEFT);
        xboxController.getAxis(XBoxAxes.LEFTY).setDeadband(DEADBANDLEFT);
        xboxController.getAxis(XBoxAxes.RIGHTX).setDeadband(DEADBANDRIGHT);
        xboxController.getButton(XBoxButtons.RBUMPER).setMode(ButtonTypes.MOMENTARY);
        xboxController.getButton(XBoxButtons.START).setMode(ButtonTypes.RISING);
        xboxController.getButton(XBoxButtons.BACK).setMode(ButtonTypes.RISING);
    }
    @Override
    public HIDAxis getDriveX() {
        return xboxController.getAxis(XBoxAxes.LEFTX);
    }

    @Override
    public HIDAxis getDriveY() {
        return xboxController.getAxis(XBoxAxes.LEFTY);
    }

    @Override
    public HIDAxis getDriveZ() {
        return xboxController.getAxis(XBoxAxes.RIGHTX);
    }

    @Override
    public HIDAxis getLeftAxis() {
        return xboxController.getAxis(XBoxAxes.LEFTY);
    }

    @Override
    public HIDAxis getRightAxis() {
        return xboxController.getAxis(XBoxAxes.LEFTX);
    }

    @Override
    public HIDButton getRearRightButton() {
        return xboxController.getButton(XBoxButtons.B);
    }

    @Override
    public HIDButton getRearLeftButton() {
        return xboxController.getButton(XBoxButtons.A);
    }

    @Override
    public HIDButton getFrontRightButton() {
        return xboxController.getButton(XBoxButtons.Y);
    }

    @Override
    public HIDButton getFrontLeftButton() {
        return xboxController.getButton(XBoxButtons.X);
    }

    @Override
    public HIDButton getInvertButton() {
        return xboxController.getButton(XBoxButtons.LBUMPER);
    }

    @Override
    public HIDButton getFireButton() {
        return xboxController.getButton(XBoxButtons.A);
    }

    @Override
    public HIDAxis getCollectorAxis() {
        return xboxController.getAxis(XBoxAxes.RTRIGGER);
    }

    @Override
    public HIDButton getRegurgitateButton() {
        return xboxController.getButton(XBoxButtons.X);
    }

    @Override
    public HIDButton getClimbButton() {
        return xboxController.getButton(XBoxButtons.START);
    }

    @Override
    public HIDButton emergencyStopClimb() {
        return xboxController.getButton(XBoxButtons.BACK);
    }
}

