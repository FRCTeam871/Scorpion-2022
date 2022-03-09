package com.team871.subsystem;

import com.team871.configuration.IRobot;
import com.team871.hid.HIDAxis;
import com.team871.hid.HIDButton;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class Collector {
    private final MotorController collectorMotor;

    public Collector(IRobot robot) {
        collectorMotor = robot.getCollectorMotor();
    }

    public void activateCollector(HIDAxis collectorAxis) {
        collectorMotor.set(collectorAxis.getValue());
    }

    public void invertCollector (HIDButton regurgitateButton) {
        if(regurgitateButton.getValue()) {
            collectorMotor.set(-.75);
        }
    }
}
