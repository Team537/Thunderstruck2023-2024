package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto-RED-BACK-CORNER")

public class AutoRedBackCorner extends Auto {

    @Override
    public void runOpMode() {
        runAutoFromParameters(Alliance.RED,StartPosition.BACKSTAGE,EndPosition.CORNER);
    }

}
