package com.team871.configuration;

import com.team871.hid.AxisID;

/**
 * Maps all the axes on the X56 joystick controller to their axis numbers in
 * software.
 *
 * @author Julianna
 */
public enum X56JoystickAxes implements AxisID {

    X_AXIS(0),
    Y_AXIS(1),
    CSTICk_X_AXIS(2),
    CSTICK_Y_AXIS(3),
    Z_AXIS(4);


    private int value;

    X56JoystickAxes(int num) {
        value = num;
    }

    /**
     * Returns the mapping to the axis number in software.
     *
     * @return Int containing the axis number
     */
    int getValue() {
        return value;
    }

    @Override
    public String getName() {
        return toString();
    }

    @Override
    public int getId() {
        return value;
    }
}
