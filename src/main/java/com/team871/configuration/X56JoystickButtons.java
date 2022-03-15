package com.team871.configuration;

import com.team871.hid.ButtonID;

/**
 * Maps all the physical buttons on the X56 joystick Controller to their button
 * numbers in software.
 *
 * @author Julianna
 */
public enum X56JoystickButtons implements ButtonID {

    TRIGGER(1),

    A(2),

    B(3),

    C(4),

    D(5),

    PINKIE_TRIGGER(6),

    H1_UP(7),

    H1_RIGHT(8),

    H1_DOWN(9),

    H1_LEFT(10),

    H2_UP(11),

    H2_RIGHT(12),

    H2_DOWN(13),

    H2_LEFT(14);


    /**
     * the value
     */
    private int value;

    X56JoystickButtons(int num) {
        value = num;
    }

    /**
     * Returns the mapping to the button number in software.
     *
     * @return int containing the button number
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
