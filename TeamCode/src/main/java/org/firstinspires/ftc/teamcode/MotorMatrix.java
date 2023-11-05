package org.firstinspires.ftc.teamcode;

public class MotorMatrix {

    //defining variables of motor values
    double lf = 0;
    double rf = 0;
    double rb = 0;
    double lb = 0;

    public MotorMatrix() {}

    /**
     * creates a motor matrix
     * @param lf left front motor power
     * @param rf right front motor power
     * @param rb right back motor power
     * @param lb left back motor power
     */
    public MotorMatrix(double lf, double rf, double rb, double lb) {
        this.lf = lf;
        this.rf = rf;
        this.rb = rb;
        this.lb = lb;
    }

    /**
     * calculating motor values from cartesian inputs <x,y> for linear velocity vector and rx for angular velocity
     * @param linear the linear velocity vector
     * @param rx the magnitude of the rotational vector
     * @param botHeading the orientation of the robot (in radians)
     */
    public void setMotorMatrixFromCartesian(Vector linear, double rx, double botHeading) {

        //reducing <x,y> to a unit vector if its magnitude exceeds 1
        //this is important because quick joystick rotations can result in unpredictable values and the robot not turning in a correct direction
        double mag = linear.magnitude();
        if (mag > 1) {
            linear = linear.unit();
        }

        //clamping rx to the domain [-1,1]
        if (Math.abs(rx) > 1) {
            rx = rx/Math.abs(rx);
        }

        //revolving x and y around the origin with -botHeading degrees angles. This makes the code field centric
        double xRot = linear.x * Math.cos(botHeading) - linear.y * Math.sin(botHeading);
        double yRot = linear.x * Math.sin(botHeading) + linear.y * Math.cos(botHeading);

        //calculating the values used for linear velocity of the motors
        double cosineMove = (yRot + xRot)/Math.sqrt(2);
        double sineMove = (yRot - xRot)/Math.sqrt(2);

        double cosinePivot;
        double sinePivot;

        //because the function divides by the magnitude, we have to run two functions if mag = 0 to avoid dividing by zero. Just so you know, the limit of the second function as s or c approaches 0 is 1
        if (mag == 0) {

            //setting the rotation speeds to the right stick x variable
            cosinePivot = rx;
            sinePivot = rx;

        } else {

            //setting the rotation speeds to the right stick x variable multiplied by a changing function which ensures that the motor speed can't exceed 0, resulting in smoother movement
            cosinePivot = rx * ((1 - mag) + (sineMove * sineMove) / (2 * mag));
            sinePivot = rx * ((1 - mag) + (cosineMove * cosineMove) / (2 * mag));

        }

        //adding or subtracting the desired linear velocities which will give constant values when added together
        lf = cosineMove + cosinePivot;
        rf = sineMove - sinePivot;
        rb = cosineMove - cosinePivot;
        lb = sineMove + sinePivot;

    }

}
