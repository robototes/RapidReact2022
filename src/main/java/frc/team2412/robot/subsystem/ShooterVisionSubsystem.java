package frc.team2412.robot.subsystem;

import static frc.team2412.robot.subsystem.ShooterVisionSubsystem.ShooterVisionConstants.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Hardware;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ShooterVisionSubsystem extends SubsystemBase implements Loggable {
    public static class ShooterVisionConstants {
        public static final double LIMELIGHT_HEIGHT_OFFSET = 39;
        public static final double RIM_HEIGHT = 104; // 8ft8in
        public static final double HEIGHT_TO_RIM = RIM_HEIGHT - LIMELIGHT_HEIGHT_OFFSET;
        public static final double HUB_RADIUS = 4 * 12 / 2;
        public static final double LIMELIGHT_ANGLE_OFFSET = Math.toDegrees(Math.atan2(HEIGHT_TO_RIM, 360 - HUB_RADIUS)); // 10.95
    }

    public NetworkTable limelight;

    private ShooterVisionSubsystem() {
        limelight = NetworkTableInstance.getDefault().getTable(Hardware.HardwareConstants.LIMELIGHT);
    }

    @Override
    public void periodic() {

    }

    public boolean hasTarget() {
        return limelight.getEntry("tv").getDouble(0) == 1;
    }

    // x-axis
    @Log(name = "Yaw")
    public double getYaw() {
        return limelight.getEntry("tx").getDouble(0);
    }

    // returns in inches
    @Log(name = "Distance")
    public double getDistance() {
        double distance = HEIGHT_TO_RIM / Math.tan(Math.toRadians(getAdjustedPitch()));
        return distance + HUB_RADIUS;
    }

    @Log(name = "Pitch from horizontal")
    public double getAdjustedPitch() {
        return LIMELIGHT_ANGLE_OFFSET + getPitch();
    }

    // y-axis
    @Log(name = "Raw limelight pitch")
    public double getPitch() {
        return limelight.getEntry("ty").getDouble(0);
    }

    public void setLedOn() {
        limelight.getEntry("ledMode").setValue("1");
    }

    public void setLedOff() {
        limelight.getEntry("ledMode").setValue("0");
    }

    // Singleton
    public static final ShooterVisionSubsystem instance = new ShooterVisionSubsystem();
}
