package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Systems.Hardware.Robot;

@TeleOp(name = "Initialize")

public class Initialize extends LinearOpMode {

    Robot robot = new Robot(this);

    public void runOpMode() {
        robot.initializeRobot();
        waitForStart();
        robot.arm.shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeIsActive()) {
        }

    }

}
