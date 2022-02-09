package frc.robot;

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
    public DoubleSolenoid getKickPiston() {
        return dummySolenoid;
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
    public Gyro getGyro() {
        return null;
    }

    @Override
    public Encoder getShooterEncoder() {
        return null;
    }
}
