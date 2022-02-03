package frc.team2412.robot.util;

public class ShooterDataDistancePoint {
    private final double distance, angle, power;

    public ShooterDataDistancePoint(double distance, double angle, double power) {
        this.distance = distance;
        this.angle = angle;
        this.power = power;
    }

    public double getDistance() {
        return distance;
    }

    public double getAngle() {
        return angle;
    }

    public double getPower() {
        return power;
    }
}
