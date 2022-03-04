package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.team871.io.sensor.DigitalLimitSwitch;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class StupidRobot implements IRobot{
    private MotorController frontLeftMotor;
    private MotorController frontRightMotor;
    private MotorController dummyMotorController;
    private DoubleSolenoid dummySolenoid;
    private  DigitalInput dummySensor;

    public StupidRobot() {
        dummyMotorController = new PWMSparkMax(0);
        dummySolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);
        dummySensor = new DigitalInput(0);
    }
    @Override
    public MotorController getFrontLeftMotor() {
        return dummyMotorController;
    }

    @Override
    public MotorController getFrontRightMotor() {
        return dummyMotorController;
    }

    @Override
    public MotorController getRearLeftMotor() {
        return dummyMotorController;
    }

    @Override
    public MotorController getRearRightMotor() {
        return dummyMotorController;
    }

    @Override
    public MotorController getShooterMotor() {
        return dummyMotorController;
    }

    @Override
    public MotorController getCollectorMotor() {
        return dummyMotorController;
    }

    @Override
    public void update(IController iController) {
    }

    @Override
    public DigitalInput getSecondBallSensor() {
        return dummySensor;
    }

    @Override
    public DigitalInput getFirstBallSenor() {
        return dummySensor;
    }

    @Override
    public MotorController getGateKeepingMotor() {
        return dummyMotorController;
    }

    @Override
    public AHRS getGyro() {
        return null;
    }

    @Override
    public Encoder getShooterEncoder() {
        return null;
    }

    @Override
    public double getValueThing() {
        return 0;
    }

    @Override
    public MotorController getLeftGrabArm() {
        return null;
    }

    @Override
    public MotorController getRightGrabArm() {
        return null;
    }

    @Override
    public MotorController getSwingArmRight() {
        return null;
    }

    @Override
    public MotorController getSwingArmLeft() {
        return null;
    }

    @Override
    public DigitalLimitSwitch getGrabHookedLimitSwitch() {
        return null;
    }

    @Override
    public DigitalLimitSwitch getSwingHookedLimitSwtich() {
        return null;
    }

    @Override
    public DigitalLimitSwitch getFullExtendLimitSwitch() {
        return null;
    }

    @Override
    public DigitalLimitSwitch getFullRetractLimitSwitch() {
        return null;
    }

    @Override
    public DigitalLimitSwitch getSwingForwardLimitSwitch() {
        return null;
    }

    @Override
    public DigitalLimitSwitch getSwingBackLimitSwitch() {
        return null;
    }
}
