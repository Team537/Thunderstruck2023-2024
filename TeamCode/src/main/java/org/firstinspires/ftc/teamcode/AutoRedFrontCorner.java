package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto-RED-FRONT-CORNER")

public class AutoRedFrontCorner extends LinearOpMode {

    @Override
    public void runOpMode() {
        new Auto().runAutoFromParameters(Alliance.RED,StartPosition.FRONTSTAGE,EndPosition.CORNER);
    }

}
