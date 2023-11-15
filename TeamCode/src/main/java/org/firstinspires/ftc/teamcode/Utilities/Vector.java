package org.firstinspires.ftc.teamcode.Utilities;

public class Vector {

    public double x;
    public double y;

    /**
     * unit vector
     * @return Vector equal to unit vector
     */
    public Vector unit() {
        return new Vector(this.x * this.magnitude(),this.y * this.magnitude());
    }

    public double magnitude() {
        return Math.sqrt( (this.x * this.x) + (this.y * this.y) );
    }

    public double angle() {
        return Math.atan2(this.y,this.x);
    }

    public double[] components() {
        return new double[] {
            this.x,
            this.y,
        };
    }

    public String string() {
        return "x : " + this.x + " y : " + this.y;
    }

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public static Vector fromPolar(double r, double theta) {
        return new Vector(r * Math.cos(theta),r * Math.sin(theta));
    }

    public static Vector add(Vector a, Vector b) {
        return new Vector(a.x + b.x, a.y + b.y);
    }

    public static Vector subtract(Vector a, Vector b) {
        return new Vector(a.x - b.x, a.y - b.y);
    }

    public static Vector multiply(Vector v, double scalar) {
        return new Vector(v.x * scalar, v.y * scalar);
    }

    public static Vector divide(Vector v, double scalar) {
        return new Vector(v.x / scalar, v.y / scalar);
    }

    public static Vector invert(Vector v) {
        return new Vector(-v.x, -v.y);
    }

    public static double dot(Vector a, Vector b) {
        return (a.x * b.x) + (a.y * b.y);
    }

    public static double cross(Vector a, Vector b) {
        return (a.x * b.y) - (a.y * b.x);
    }

    public static Vector project(Vector u, Vector v) {
        return Vector.multiply(v, ( Vector.dot(u,v) / (v.magnitude()*v.magnitude()) ) );
    }

    public static Vector rotate(Vector v, double angle) {
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);
        return new Vector((v.x * cos) - (v.y * sin),(v.x * sin) + (v.y * cos));
    }

    public static Vector rotateAround(Vector v, Vector u, double angle) {
        return Vector.add(u,Vector.rotate(Vector.subtract(v,u),angle));
    }

    public static Vector setAngle(Vector v, double angle) {
        return Vector.fromPolar(v.magnitude(), angle);
    }

}
