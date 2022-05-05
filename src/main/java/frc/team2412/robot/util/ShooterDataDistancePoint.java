package frc.team2412.robot.util;

public class ShooterDataDistancePoint {
    private final double distance, angle, RPM, timeOfFlight;

    public ShooterDataDistancePoint(double distance, double angle, double RPM, double timeOfFlight) {
        this.distance = distance;
        this.angle = angle;
        this.RPM = RPM;
        this.timeOfFlight = timeOfFlight;
        timeOfFlight = Double.NaN;
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

    public double getTimeOfFlight() {
        return timeOfFlight;
    }

    @Override
    public String toString() {
        return "{distance: " + distance + ", angle: " + angle + ", RPM: " + RPM + ", time of flight: " + timeOfFlight
                + "}";
    }
}
