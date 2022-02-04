package frc.team2412.robot.subsystem;

import static frc.team2412.robot.subsystem.ShooterVisionSubsystem.ShooterVisionConstants.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterVisionSubsystem extends SubsystemBase {

    public static class ShooterVisionConstants {

        public static double LIMELIGHT_ANGLE_OFFSET = 0;
        public static double LIMELIGHT_HEIGHT_OFFSET = 0;
        public static double RIM_HEIGHT = 104; // 8ft8in
        public static double HEIGHT_TO_RIM = RIM_HEIGHT - LIMELIGHT_HEIGHT_OFFSET;
    }

    public NetworkTable limelight;

    public ShooterVisionSubsystem() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void periodic() {

    }

    public boolean hasTarget() {
        return limelight.getEntry("tv").getDouble(0) == 1;
    }

    // x-axis
    public double getYaw() {
        return limelight.getEntry("tx").getDouble(0);
    }

    // returns in inches
    public double getDistance() {
        double distance = HEIGHT_TO_RIM / Math.tan(LIMELIGHT_ANGLE_OFFSET + getPitch());
        return distance;
    }

    // y-axis
    public double getPitch() {
        return limelight.getEntry("ty").getDouble(0);
    }

    public void setLedOn() {
        limelight.getEntry("ledMode").setValue("1");
    }

    public void setLedOff() {
        limelight.getEntry("ledMode").setValue("0");
    }

}
