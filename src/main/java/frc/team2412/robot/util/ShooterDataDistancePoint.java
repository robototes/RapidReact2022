package frc.team2412.robot.util;

public class ShooterDataDistancePoint {
    private final double distance, angle, RPM;

    public ShooterDataDistancePoint(double distance, double angle, double RPM) {
        this.distance = distance;
        this.angle = angle;
        this.RPM = RPM;
    }

    public double getDistance() {
        return distance;
    }

    public double getAngle() {
        return angle;
    }

    public double getRPM() {
        return RPM;
    }
}
