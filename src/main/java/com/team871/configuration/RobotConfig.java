package com.team871.configuration;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.team871.io.sensor.DigitalLimitSwitch;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class RobotConfig implements IRobot {
    private final CANSparkMax frontLeft;
    private final CANSparkMax frontRight;
    private final CANSparkMax rearLeft;
    private final CANSparkMax rearRight;

    private final WPI_TalonSRX shooterMotor;
    private final WPI_VictorSPX collectorMotor;
    private final WPI_VictorSPX gateKeepMotor;
    private final WPI_TalonFX leftGrabArm;
    private final WPI_TalonFX rightGrabArm;
    private final WPI_TalonSRX swingArmRight;
    private final WPI_TalonSRX swingArmLeft;

    private final AHRS gyro;
    private final Encoder encoder;

    private final DigitalInput onDeckSensor;
    private final DigitalInput loadedSensor;
    private final DigitalLimitSwitch fullExtendLimitSwitch;
    private final DigitalLimitSwitch fullRetractLimitSwitch;
    private final DigitalLimitSwitch swingForwardLimitSwitch;
    private final DigitalLimitSwitch swingBackLimitSwitch;


    public RobotConfig() {
        /* sets front left motor to CanSparkMax motor controller with device id 1 */
        frontLeft = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
        frontLeft.setInverted(true);

        /* sets front right motor to CanSparkMax motor controller with device id 2 */
        frontRight = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
        frontRight.setInverted(false);

        /* sets rear left motor to CanSparkMax motor controller with device id 3 */
        rearLeft = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
        rearLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rearLeft.setInverted(true);

        /* sets rear right motor to CanSparkMax motor controller with device id 4 */
        rearRight = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
        rearRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rearRight.setInverted(false);

        collectorMotor = new WPI_VictorSPX(5);
        collectorMotor.setInverted(true);
        collectorMotor.setNeutralMode(NeutralMode.Coast);

        shooterMotor = new WPI_TalonSRX(6);
        shooterMotor.setNeutralMode(NeutralMode.Coast);

        gateKeepMotor = new WPI_VictorSPX(7);
        gateKeepMotor.setInverted(true);
        gateKeepMotor.setNeutralMode(NeutralMode.Brake);

        gyro = new AHRS();
        //TODO: find actual channels
        encoder = new Encoder(0, 1);
        leftGrabArm = new WPI_TalonFX(9);
        leftGrabArm.setInverted(true);
        leftGrabArm.setNeutralMode(NeutralMode.Brake);
        rightGrabArm = new WPI_TalonFX(8);
        rightGrabArm.setNeutralMode(NeutralMode.Brake);

        swingArmLeft = new WPI_TalonSRX(11);
        swingArmLeft.setInverted(true);
        swingArmLeft.setNeutralMode(NeutralMode.Brake);
        swingArmRight = new WPI_TalonSRX(10);
        swingArmRight.setNeutralMode(NeutralMode.Brake);

        //TODO: find actual inputs
        fullExtendLimitSwitch = new DigitalLimitSwitch(new DigitalInput(7));
        fullRetractLimitSwitch = new DigitalLimitSwitch(new DigitalInput(3));
        swingForwardLimitSwitch = new DigitalLimitSwitch(new DigitalInput(6));
        swingBackLimitSwitch = new DigitalLimitSwitch(new DigitalInput(5));

        onDeckSensor = new DigitalInput(9);
        loadedSensor = new DigitalInput(8);
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
    public DigitalInput getLoadedSensor() {
        return loadedSensor;
    }

    @Override
    public DigitalInput getOnDeckSensor() {
        return onDeckSensor;
    }

    @Override
    public MotorController getGateKeepingMotor() {
        return gateKeepMotor;
    }

    @Override
    public AHRS getGyro() {
        return gyro;
    }

    @Override
    public Encoder getShooterEncoder() {
        return encoder;
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