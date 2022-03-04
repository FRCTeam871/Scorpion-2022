package frc.robot;

import com.team871.hid.HIDAxis;
import com.team871.hid.HIDButton;

public interface IController {
    /**
     * x-axis controls the horizontal movement of the robot
     * @return The x-axis of the drive train
     */
    HIDAxis getDriveX();
    /**
     * y-axis controls the forward and backward movement of the robot
     * @return The y-axis of the drive train
     */
    HIDAxis getDriveY();
    /**
     * z-axis controls the rotation of the robot
     * @return The z-axis of the drive train
     */
    HIDAxis getDriveZ();

    /**
     * @return The axis used to control the left side of the drive train in tank drive
     */
    HIDAxis getLeftAxis();

    /**
     * @return The axis used to control the right side of the drive train in tank drive
     */
    HIDAxis getRightAxis();

    /**
     * @return the axis controlling the vertical arms
     */
    HIDAxis getClimbGrabAxis();

    /**
     * @return the axis controlling the swing arms
     */
    HIDAxis getClimbSwingAxis();

    HIDAxis getAltClimbSwingAxis();

    /**
     * The button which enable/disables rear right motor
     *
     * @return The rear right button
     */
    HIDButton getRearRightButton();

    /**
     * The button which enables/disables rear left motor
     *
     * @return The rear left button
     */
    HIDButton getRearLeftButton();
    /**
     * The button which enables/disables front right motor
     * @return The front right button
     */
    HIDButton getFrontRightButton();
    /**
     * The button which enables/disables front left motor
     * @return The front left button
     */
    HIDButton getFrontLeftButton();
    /**
     * The button which inverts the motor that corresponds to the button pressed down
     * @return The invert button
     */
    HIDButton getInvertButton();
    /**
     * The button which enables the firing system
     * @return The fire button
     */
    HIDButton getFireButton();

    /**
     * The axis which controls the speed the cargo collector spins
     *
     * @return The collector axis
     */
    HIDAxis getCollectorAxis();

    HIDButton getRegurgitateButton();

    HIDButton getClimbButton();

    HIDButton getRevertClimbButton();
}
