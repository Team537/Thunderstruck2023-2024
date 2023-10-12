package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto-SWITCHES")

public class AutoSwitches extends LinearOpMode {

    @Override
    public void runOpMode() {
        new Auto().runAutoFromSwitches();
    }

}