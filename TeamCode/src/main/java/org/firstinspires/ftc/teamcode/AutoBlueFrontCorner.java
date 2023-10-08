package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto-BLUE-FRONT-CORNER")

public class AutoBlueFrontCorner extends LinearOpMode {

    @Override
    public void runOpMode() {
        new Auto().runAutoFromParameters(Alliance.BLUE,StartPosition.FRONTSTAGE,EndPosition.CORNER);
    }

}
