package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.TouchSensor;

public class SettingSwitches {

    //defining variables for each switch
    TouchSensor alliance;
    TouchSensor startPosition;
    TouchSensor endPosition;

    //defining functions for each touch sensor to return its boolean value
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
