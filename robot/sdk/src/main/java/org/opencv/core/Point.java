package org.opencv.core;

/** OpenCV's 2D point (image pixels when it comes from a vision processor). */
public class Point {
    public double x;
    public double y;

    public Point() {
        this(0, 0);
    }

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point clone() {
        return new Point(x, y);
    }

    public double dot(Point p) {
        return x * p.x + y * p.y;
    }

    @Override
    public boolean equals(Object obj) {
        return obj instanceof Point && ((Point) obj).x == x && ((Point) obj).y == y;
    }

    @Override
    public int hashCode() {
        return Double.hashCode(x) * 31 + Double.hashCode(y);
    }

    @Override
    public String toString() {
        return "{" + x + ", " + y + "}";
    }
}
