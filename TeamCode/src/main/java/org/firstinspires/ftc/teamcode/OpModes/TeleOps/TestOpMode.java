package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.DriveMode;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.DriveSpeed;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.FieldOfReference;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.ScoringPosition;
import org.firstinspires.ftc.teamcode.Systems.Software.SoftwareEnums.TurningMode;
import org.firstinspires.ftc.teamcode.Utilities.TargetTurn;
import org.firstinspires.ftc.teamcode.Utilities.Vector;

@TeleOp(name = "TestOpMode")

public class TestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot robot = new Robot(this);
        robot.initializeRobot();

        waitForStart();

        //loop that runs while the program is active
        while (opModeIsActive()) {

            if (gamepad1.x) {
                robot.drivetrain.lfMotor.setPower(1);
                robot.drivetrain.rfMotor.setPower(1);
                robot.drivetrain.lbMotor.setPower(1);
                robot.drivetrain.rbMotor.setPower(1);
            } else {
                robot.drivetrain.lfMotor.setPower(-1);
                robot.drivetrain.rfMotor.setPower(-1);
                robot.drivetrain.lbMotor.setPower(-1);
                robot.drivetrain.rbMotor.setPower(-1);
            }

        }

    }
}
