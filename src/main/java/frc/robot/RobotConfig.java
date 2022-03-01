package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.team871.io.sensor.DigitalLimitSwitch;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotConfig implements IRobot{
    private CANSparkMax frontLeft;
    private CANSparkMax frontRight;
    private CANSparkMax rearLeft;
    private CANSparkMax rearRight;
    private MotorController shooterMotor;
    private MotorController collectorMotor;
    private DigitalInput ballSensor1;
    private DigitalInput ballSensor2;
    private MotorController gateKeepMotor;
    private Gyro gyro;
    private Encoder encoder;
    private MotorController leftGrabArm;
    private MotorController rightGrabArm;
    private MotorController swingArmRight;
    private MotorController swingArmLeft;
    private DigitalLimitSwitch grabHookLimitSwitch;
    private DigitalLimitSwitch swingHookLimitSwitch;
    private DigitalLimitSwitch fullExtendLimitSwitch;
    private DigitalLimitSwitch fullRetractLimitSwitch;
    private DigitalLimitSwitch swingForwardLimitSwitch;
    private DigitalLimitSwitch swingBackLimitSwitch;


    public RobotConfig() {
        SmartDashboard.putNumber("ThrowerPct", 5);

        /**
         * sets front left motor to CanSparkMax motor controller with device id 1
         */
        frontLeft = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
        frontLeft.setInverted(true);
        /**
         * sets front right motor to CanSparkMax motor controller with device id 2
         */
        frontRight = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
        frontRight.setInverted(false);
        /**
         * sets rear left motor to CanSparkMax motor controller with device id 3
         */
        rearLeft = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
        rearLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rearLeft.setInverted(true);
        /**
         * sets rear right motor to CanSparkMax motor controller with device id 4
         */
        rearRight = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
        rearRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rearRight.setInverted(false);
        collectorMotor = new WPI_VictorSPX(5);
        collectorMotor.setInverted(true);
        shooterMotor = new WPI_TalonSRX(6);
        gateKeepMotor = new WPI_VictorSPX(7);
        gateKeepMotor.setInverted(true);
        gyro = new AHRS();
        //TODO: find actual channels
        encoder = new Encoder(0, 1);
        leftGrabArm = new WPI_TalonFX(9);
        leftGrabArm.setInverted(true);
        rightGrabArm = new WPI_TalonFX(8);
        swingArmLeft = new WPI_TalonSRX(11);
        swingArmLeft.setInverted(true);
        swingArmRight = new WPI_TalonSRX(10);
        //TODO: find actual inputs
//        grabHookLimitSwitch = new DigitalLimitSwitch(new DigitalInput(5));
//        swingHookLimitSwitch = new DigitalLimitSwitch(new DigitalInput(6));
//        fullExtendLimitSwitch = new DigitalLimitSwitch(new DigitalInput(7));
//        fullRetractLimitSwitch = new DigitalLimitSwitch(new DigitalInput(3));
//        swingForwardLimitSwitch = new DigitalLimitSwitch(new DigitalInput(2));
//        swingBackLimitSwitch = new DigitalLimitSwitch(new DigitalInput(2));

        //ballSensor1 = new DigitalInput(99);
        //ballSensor2 = new DigitalInput(9);
    }
    @Override
    public MotorController getFrontLeftMotor() {
        return frontLeft;
    }

    @Override
    public MotorController getFrontRightMotor() {
        return frontRight;
    }

    @Override
    public MotorController getRearLeftMotor() {
        return rearLeft;
    }

    @Override
    public MotorController getRearRightMotor() {
        return rearRight;
    }

    @Override
    public MotorController getShooterMotor() {
        return shooterMotor;
    }

    @Override
    public MotorController getCollectorMotor() {
        return collectorMotor;
    }

    @Override
    public void update(IController iController) {
//        frontLeft.setEnabled(iController.getFrontLeftButton().getValue());
//        frontRight.setEnabled(iController.getFrontRightButton().getValue());
//        rearLeft.setEnabled(iController.getRearLeftButton().getValue());
//        rearRight.setEnabled(iController.getRearRightButton().getValue());

        if (iController.getFrontLeftButton().getValue() && iController.getInvertButton().getValue()) {
            frontLeft.setInverted(!frontLeft.getInverted());
        }
        if (iController.getFrontRightButton().getValue() && iController.getInvertButton().getValue()){
            frontRight.setInverted(!frontRight.getInverted());
        }
        if (iController.getRearLeftButton().getValue() && iController.getInvertButton().getValue()){
            rearLeft.setInverted(!rearLeft.getInverted());
        }
        if (iController.getRearRightButton().getValue() && iController.getInvertButton().getValue()){
            rearRight.setInverted(!rearRight.getInverted());
        }
    }

    @Override
    public DigitalInput getSecondBallSensor() {
        return ballSensor2;
    }

    @Override
    public DigitalInput getFirstBallSenor() {
        return ballSensor1;
    }

    @Override
    public MotorController getGateKeepingMotor() {
        return gateKeepMotor;
    }

    @Override
    public Gyro getGyro() {
        return gyro;
    }

    @Override
    public Encoder getShooterEncoder() {
        return encoder;
    }

    @Override
    public double getValueThing() {
        return SmartDashboard.getNumber("ThrowerPct", 0);
    }

    @Override
    public MotorController getLeftGrabArm() {
        return leftGrabArm;
    }

    @Override
    public MotorController getRightGrabArm() {
        return rightGrabArm;
    }

    @Override
    public MotorController getSwingArmLeft() {
        return swingArmLeft;
    }

    @Override
    public MotorController getSwingArmRight() {
        return swingArmRight;
    }

    @Override
    public DigitalLimitSwitch getGrabHookedLimitSwitch() {
        return grabHookLimitSwitch;
    }

    @Override
    public DigitalLimitSwitch getSwingHookedLimitSwtich() {
        return swingHookLimitSwitch;
    }

    @Override
    public DigitalLimitSwitch getFullExtendLimitSwitch() {
        return fullExtendLimitSwitch;
    }

    @Override
    public DigitalLimitSwitch getFullRetractLimitSwitch() {
        return fullRetractLimitSwitch;
    }

    @Override
    public DigitalLimitSwitch getSwingForwardLimitSwitch() {
        return swingForwardLimitSwitch;
    }

    @Override
    public DigitalLimitSwitch getSwingBackLimitSwitch() {
        return swingBackLimitSwitch;
    }
}