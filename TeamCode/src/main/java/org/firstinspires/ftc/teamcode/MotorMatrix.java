package org.firstinspires.ftc.teamcode;

public class MotorMatrix {

    double lf = 0;
    double rf = 0;
    double rb = 0;
    double lb = 0;

    public void setMotorMatrixFromCartesian(double x, double y, double rx, double botHeading) {

        double mag = Math.sqrt(x*x + y*y);
        if (mag > 1) {
            x = x/mag;
            y = y/mag;
        }

        if (Math.abs(rx) > 1) {
            rx = rx/Math.abs(rx);
        }

        double xRot = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double yRot = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double cosineMove = (yRot + xRot)/Math.sqrt(2);
        double sineMove = (yRot - xRot)/Math.sqrt(2);

        double cosinePivot;
        double sinePivot;

        //because the function divides by the magnitude, we have to run two functions if mag = 0 to avoid dividing by zero. Just so you know, the limit of the second function as x approaches 0 is 1, so they are essentially identical
        if (mag == 0) {

            //setting the rotation speeds to the right stick x variable
            cosinePivot = rx;
            sinePivot = rx;

        } else {

            //setting the rotation speeds to the right stick x variable multiplied by a changing function which ensures that the motor speed can't exceed 0, resulting in smoother movement
            cosinePivot = rx * ((1 - mag) + (sineMove * sineMove) / (2 * mag));
            sinePivot = rx * ((1 - mag) + (cosineMove * cosineMove) / (2 * mag));

        }

        lf = cosineMove + cosinePivot;
        rf = sineMove - sinePivot;
        rb = cosineMove - cosinePivot;
        lb = sineMove + sinePivot;

    }

}
