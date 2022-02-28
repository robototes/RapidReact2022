package frc.team2412.robot.util;

public class ShooterDataDistancePoint{
    private final double distance, angle, power;

    public ShooterDataDistancePoint(double dist, double ang, double pow){
        distance = dist;
        angle = ang;
        power = pow;
    }
    public double getDistance(){
        return distance;
    }
    public double getAngle(){
        return angle;
    }
    public double getPower(){
        return power;
    }
}
