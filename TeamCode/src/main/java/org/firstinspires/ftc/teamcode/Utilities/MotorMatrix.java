package org.firstinspires.ftc.teamcode.Utilities;

public class MotorMatrix {

    //defining variables of motor values
    public double lf = 0;
    public double rf = 0;
    public double rb = 0;
    public double lb = 0;

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
     * @param multiplier a scalar all motor speeds will be multiplied by
     */
    public void setMotorMatrixFromCartesian(Vector linear, double rx, double botHeading, double multiplier) {

        //reducing <x,y> to a unit vector if its magnitude exceeds 1
        //this is important because quick joystick rotations can result in unpredictable values and the robot not turning in a correct direction
        Vector clampedLinear;

        if (linear.magnitude() > 1) {
            clampedLinear = linear.unit();
        } else {
            clampedLinear = linear;
        }

        //clamping rx to the domain [-1,1]
        if (Math.abs(rx) > 1) {
            rx = rx/Math.abs(rx);
        }

        //revolving x and y around the origin with -botHeading degrees angles. This makes the code field centric

        Vector rotatedLinear = Vector.rotate(clampedLinear,botHeading);

        //calculating the values used for linear velocity of the motors
        double cosineMove = (rotatedLinear.x - rotatedLinear.y)/Math.sqrt(2);
        double sineMove = (rotatedLinear.x + rotatedLinear.y)/Math.sqrt(2);

        double cosinePivot;
        double sinePivot;

        double mag = new Vector(cosineMove,sineMove).magnitude();

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
        lf = (cosineMove - cosinePivot) * multiplier;
        rf = (sineMove + sinePivot) * multiplier;
        rb = (cosineMove + cosinePivot) * multiplier;
        lb = (sineMove - sinePivot) * multiplier;

    }

}
