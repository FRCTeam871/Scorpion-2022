package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotConfig implements IRobot{
    private CANSparkMax frontLeft;
    private CANSparkMax frontRight;
    private CANSparkMax rearLeft;
    private CANSparkMax rearRight;
    private MotorController shooterMotor;
    private MotorController collectorMotor;
    private DoubleSolenoid kickPiston;
    private DigitalInput ballSensor1;
    private DigitalInput ballSensor2;
    private MotorController gateKeepMotor;
    private Gyro gyro;
    private Encoder encoder;

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

        //TODO: confirm parameters for Double Solenoid
        // kickPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);

        //ballSensor1 = new DigitalInput(99);
        //ballSensor2 = new DigitalInput(9);
        //gateKeepMotor = new WPI_VictorSPX(7);
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
    public DoubleSolenoid getKickPiston() {
        return null;
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
}