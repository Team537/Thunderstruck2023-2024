package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.TouchSensor;

public class SettingSwitches {
    TouchSensor alliance;
    TouchSensor startPosition;
    TouchSensor endPosition;

    public void SettingSwitches(TouchSensor alliance,TouchSensor startPosition,TouchSensor endPosition) {
        this.alliance = alliance;
        this.startPosition = startPosition;
        this.endPosition = endPosition;
    }

    public boolean getAlliance() {
        return alliance.isPressed();
    }

    public boolean getStartPosition() {
        return startPosition.isPressed();
    }

    public boolean getEndPosition() {
        return endPosition.isPressed();
    }

}
