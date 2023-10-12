package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivetrain {

    //defining individual motor attributes
    DcMotor lfMotor;
    DcMotor rfMotor;
    DcMotor rbMotor;
    DcMotor lbMotor;

    //runs drivetrain with a MotorMatrix parameter
    public void runDrivetrain(MotorMatrix motorMatrix) {
        lfMotor.setPower(motorMatrix.lf);
        rfMotor.setPower(motorMatrix.rf);
        lbMotor.setPower(motorMatrix.lb);
        rbMotor.setPower(motorMatrix.rb);
    }

    //runs drivetrain with parameters; uses a MotorMatrix to calculate parameters
    public void runDrivetrainFromCartesian(double x, double y, double rx, double botHeading) {
        MotorMatrix motorMatrix = new MotorMatrix();
        motorMatrix.setMotorMatrixFromCartesian(x,y,rx,botHeading);
        lfMotor.setPower(motorMatrix.lf);
        rfMotor.setPower(motorMatrix.rf);
        lbMotor.setPower(motorMatrix.lb);
        rbMotor.setPower(motorMatrix.rb);
    }

    //stops all drivetrain motors
    public void stop() {
        lfMotor.setPower(0);
        rfMotor.setPower(0);
        lbMotor.setPower(0);
        rbMotor.setPower(0);
    }

    //runs all drivetrain motors in a way so no net velocity is applied to the robot
    public void rave() {
        lfMotor.setPower(1);
        rfMotor.setPower(1);
        lbMotor.setPower(-1);
        rbMotor.setPower(-1);
    }

}
