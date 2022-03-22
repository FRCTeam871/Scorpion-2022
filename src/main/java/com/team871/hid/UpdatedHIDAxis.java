package com.team871.hid;

import edu.wpi.first.wpilibj.Joystick;

public class UpdatedHIDAxis extends HIDAxis {
    private boolean isInverted;
    private HIDAxis axis;

    public UpdatedHIDAxis(HIDAxis axis) {
        super(null, null);
        isInverted = false;
        this.axis = axis;
    }

    public UpdatedHIDAxis(HIDAxis axis, boolean isInverted) {
        super(null, null);
        this.isInverted = isInverted;
        this.axis = axis;
    }

    public void setInverted(boolean isInverted) {
        this.isInverted = isInverted;
    }

    public double getValue() {
        return isInverted ? axis.getValue() * -1 : axis.getValue();
    }

    public double getRaw() {
        return axis.getValue();
    }
}
