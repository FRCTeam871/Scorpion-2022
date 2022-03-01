package frc.robot;

import com.team871.hid.HIDAxis;
import com.team871.hid.HIDButton;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class Collector {
    private MotorController collectorMotor;
    private DigitalInput ballSensor1;
    private DigitalInput ballSensor2;

    public Collector(IRobot robot) {
        collectorMotor = robot.getCollectorMotor();
        ballSensor1 = robot.getFirstBallSenor();
        ballSensor2 = robot.getSecondBallSensor();
    }
    public void activateCollector(HIDAxis collectorAxis) {
        collectorMotor.set(collectorAxis.getValue());
    }

    public void invertCollector (HIDButton regurgitateButton) {
        if(regurgitateButton.getValue()){
            collectorMotor.set(-.75);
        }
    }
}
