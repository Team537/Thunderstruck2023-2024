package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Systems.Software.Auto;

@Autonomous(name="Auto-SENSORS")

public class AutoSensors extends Auto {

    @Override
    public void runOpMode() {
        runAutoFromSensors();
    }

}