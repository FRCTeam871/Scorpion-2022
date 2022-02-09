package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class ToggleableMotorController implements MotorController {
    boolean enabled = true;
    MotorController motorController;
    public ToggleableMotorController(MotorController motorController) {
        this.motorController = motorController;
    }
    @Override
    public void set(double speed) {
        if (enabled) {
            motorController.set(speed);
        } else {
            motorController.set(0);
        }
    }
    @Override
    public double get() {
        if (enabled) {
            return motorController.get();
        }
        return 0;
    }
    @Override
    public void setInverted(boolean isInverted) {
        motorController.setInverted(isInverted);
    }
    @Override
    public boolean getInverted() {
        return motorController.getInverted();
    }

    @Override
    public void disable() {
        motorController.disable();
    }
    @Override
    public void stopMotor() {
        motorController.stopMotor();
    }
    public void setEnabled (boolean enabled) {
        this.enabled = enabled;
    }
}
