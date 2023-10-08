package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto-RED-FRONT-MIDDLE")

public class AutoRedFrontMiddle extends LinearOpMode {

    @Override
    public void runOpMode() {
        new Auto().runAutoFromParameters(Alliance.RED,StartPosition.FRONTSTAGE,EndPosition.MIDDLE);
    }

}
