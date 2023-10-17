package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto-BLUE-BACK-MIDDLE")

public class AutoBlueBackMiddle extends Auto {

    @Override
    public void runOpMode() {
        runAutoFromParameters(Alliance.BLUE,StartPosition.BACKSTAGE,EndPosition.MIDDLE);
    }

}
