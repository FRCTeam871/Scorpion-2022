package frc.robot;

import com.team871.io.sensor.DigitalLimitSwitch;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public interface IRobot {
    /**
     * Get front left motor is the front left motor for the drive system
     * @return the motor controller responsible for moving the front left wheel
     */
    public MotorController getFrontLeftMotor();
    /**
     *  Get front right motor is the front right motor for the drive system
     * @return the motor controller responsible for moving the front right wheel
     */
    public MotorController getFrontRightMotor();
    /**
     *  Get rear left motor is the rear left motor for the drive system
     * @return the motor controller responsible for moving the rear left wheel
     */
    public MotorController getRearLeftMotor();
    /**
     *  Get Rear Right Motor is the rear right motor for the drive system
     * @return the motor controller responsible for moving the rear right wheel
     */
    public MotorController getRearRightMotor();
    /**
     * @return the motor controller responsible for firing the cargo
     */
    public MotorController getShooterMotor();
    /**
     * @return the motor controller responsible for collecting the cargo
     */
    public MotorController getCollectorMotor();
    /**
     * U P D A T E S
     * @param iController
     */
    public void update(IController iController);
    /**
     * @return sensor responsible for sensing if a ball is in the second position (closer to collector)
     */
    public DigitalInput getSecondBallSensor();
    /**
     * @return sensor responsible for sensing if a ball is in the first position (closer to launcher)
     */
    public DigitalInput getFirstBallSenor();
    /**
     * @return motor that holds the balls before it doesn't and it actually just kinda sorta pushes the balls into the big spinny wheel for supreme yeetage JEEPERS
     */
    public MotorController getGateKeepingMotor();

    //TODO: add 2 camera sensors for the shooter and for the intake

    /**
     * @return the gyro of the drivetrain
     */
    Gyro getGyro();

    /**
     * measure the rpm of the firing wheel on the shooter
     *
     * @return the encoder of the shooter
     */
    Encoder getShooterEncoder();

    double getValueThing();

    /**
     * gets the motor responsible for moving the arms which initially grab the bar
     *
     * @return the grab arms
     */
    MotorController getLeftGrabArm();

    /**
     * gets the motor responsible for moving the arms which initially grab the bar
     *
     * @return the grab arms
     */
    MotorController getRightGrabArm();

    /**
     * @return the rightmost motor controlling the Swing Arms
     */
    MotorController getSwingArmRight();

    /**
     * @return the leftmost motor controlling the Swing Arms
     */
    MotorController getSwingArmLeft();

    /**
     * gets the limit switch responsible for telling if the grab arms are hooked to the bar
     *
     * @return grab hooked limit switch
     */
    DigitalLimitSwitch getGrabHookedLimitSwitch();

    /**
     * gets the limit switch responsible for telling if the hang arms are hooked to the bar
     *
     * @return hang hooked limit switch
     */
    DigitalLimitSwitch getSwingHookedLimitSwtich();

    /**
     * gets the limit switch responsible for telling if the grab arm is fully extended
     *
     * @return full extend limit switch
     */
    DigitalLimitSwitch getFullExtendLimitSwitch();

    /**
     * gets the limit switch responsible for telling if the grab arm is fully retracted
     *
     * @return full retract limit switch
     */
    DigitalLimitSwitch getFullRetractLimitSwitch();

    /**
     * located closer to the back, but sees when the robot is leaning forwards
     *
     * @return limit switch responsible for detecting when the robot is leaning forwards
     */
    DigitalLimitSwitch getSwingForwardLimitSwitch();

    /**
     * located closer to the front, but is responsible for detecting when the robot is leaning backwards
     *
     * @return limit switch responsible for detecting when the robot is leaning backwards
     */
    DigitalLimitSwitch getSwingBackLimitSwitch();
}
