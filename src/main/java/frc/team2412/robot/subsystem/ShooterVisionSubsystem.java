package frc.team2412.robot.subsystem;

import static frc.team2412.robot.subsystem.ShooterVisionSubsystem.ShooterVisionConstants.*;

import static frc.team2412.robot.Hardware.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class ShooterVisionSubsystem extends SubsystemBase implements Loggable {
    public static class ShooterVisionConstants {
        // Dimensions are in inches
        public static final double LIMELIGHT_HEIGHT_OFFSET = 37.5;
        public static final double RIM_HEIGHT = 104; // 8ft8in
        public static final double HEIGHT_TO_RIM = RIM_HEIGHT - LIMELIGHT_HEIGHT_OFFSET;
        public static final double HUB_RADIUS = 24;
        // Angles are in degrees
        public static final double LIMELIGHT_ANGLE_OFFSET = Math.toDegrees(Math.atan2(HEIGHT_TO_RIM, 360 - HUB_RADIUS)); // 10.95

        // -0.766666 limelight crosshair offset (3/19 update)
    }

    public NetworkTable limelight;

    public ShooterVisionSubsystem() {
        limelight = NetworkTableInstance.getDefault().getTable(LIMELIGHT);
        setCompPipline();
    }

    public boolean hasTarget() {
        return limelight.getEntry("tv").getDouble(0) == 1;
    }

    /**
     * Returns the yaw from the limelight to the (estimated) center of the hub.
     * Note: The limelight returns measurements relative to the center of the targets in its field of
     * view, which may differ from the center of the hub.
     *
     * @return The yaw (horizontal rotation) in degrees.
     */
    @Log(name = "Yaw")
    public double getYaw() {
        return limelight.getEntry("tx").getDouble(0);
    }

    /**
     * Returns the distance from the limelight to the (estimated) center of the hub.
     * Note: The limelight returns measurements relative to the center of the targets in its field of
     * view, which may differ from the center of the hub.
     *
     * @return The distance in inches.
     */
    @Log(name = "Distance")
    public double getDistance() {
        double distanceToHubRim = HEIGHT_TO_RIM / Math.tan(Math.toRadians(getAdjustedPitch()));
        return distanceToHubRim + HUB_RADIUS;
    }

    /**
     * Returns the pitch from the horizontal plane to the hub.
     *
     * @return The adjusted pitch (vertical rotation) in degrees.
     */
    @Log(name = "Pitch from horizontal")
    public double getAdjustedPitch() {
        return getPitch() + LIMELIGHT_ANGLE_OFFSET;
    }

    /**
     * Returns the raw pitch value from the limelight.
     *
     * @return The raw pitch (vertical rotation) in degrees.
     */
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

    public void setCompPipline() {
        limelight.getEntry("pipeline").setNumber(0);
    }

    // @Config(name = "set Pipeline", defaultValueNumeric = 1)
    // public void setPipeline(int pipelineNum) {
    //     limelight.getEntry("pipeline").setNumber(pipelineNum);
    // }

    // @Log(name = "current pipeline")
    // public int getPipeline() {
    //     return (int) limelight.getEntry("pipeline").getNumber(0);
    // }

}
