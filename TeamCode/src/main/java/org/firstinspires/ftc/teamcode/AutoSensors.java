package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto-SENSORS")

public class AutoSensors extends LinearOpMode {

    @Override
    public void runOpMode() {
        new Auto().runAutoFromSensors();
    }

}