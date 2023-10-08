package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivetrain {
    DcMotor lfMotor;
    DcMotor rfMotor;
    DcMotor rbMotor;
    DcMotor lbMotor;

    public Drivetrain(DcMotor lfMotor, DcMotor rfMotor, DcMotor lbMotor, DcMotor rbMotor) {
        this.lfMotor = lfMotor;
        this.rfMotor = rfMotor;
        this.lbMotor = lbMotor;
        this.rbMotor = rbMotor;
    }

    public void runDrivetrain(@NonNull MotorMatrix motorMatrix) {
        lfMotor.setPower(motorMatrix.lf);
        rfMotor.setPower(motorMatrix.rf);
        lbMotor.setPower(motorMatrix.lb);
        rbMotor.setPower(motorMatrix.rb);
    }

    public void runDrivetrainFromCartesian(double x, double y, double rx, double botHeading) {
        MotorMatrix motorMatrix = new MotorMatrix();
        motorMatrix.setMotorMatrixFromCartesian(x,y,rx,botHeading);
        lfMotor.setPower(motorMatrix.lf);
        rfMotor.setPower(motorMatrix.rf);
        lbMotor.setPower(motorMatrix.lb);
        rbMotor.setPower(motorMatrix.rb);
    }

    public void stop() {
        lfMotor.setPower(0);
        rfMotor.setPower(0);
        lbMotor.setPower(0);
        rbMotor.setPower(0);
    }

}
