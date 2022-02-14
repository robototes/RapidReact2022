package frc.team2412.robot.subsystem;

import static frc.team2412.robot.subsystem.ShooterVisionSubsystem.ShooterVisionConstants.*;

import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterVisionSubsystem extends SubsystemBase {
    public static class ShooterVisionConstants {
        public static final double LIMELIGHT_ANGLE_OFFSET = 0;
        public static final double LIMELIGHT_HEIGHT_OFFSET = 0;
        public static final double RIM_HEIGHT = 104; // 8ft8in
        public static final double HEIGHT_TO_RIM = RIM_HEIGHT - LIMELIGHT_HEIGHT_OFFSET;
        public static final Rotation2 STARTING_ROBOT_ROTATION = Rotation2.ZERO; // Placeholder
    }

    public NetworkTable limelight;

    private final Gyroscope gyro;

    private final Rotation2 gyroAdjustmentAngle;

    public ShooterVisionSubsystem(Gyroscope gyro) {
        this.limelight = NetworkTableInstance.getDefault().getTable("limelight");
        this.gyro = gyro;
        this.gyroAdjustmentAngle = STARTING_ROBOT_ROTATION.rotateBy(gyro.getUnadjustedAngle().inverse());
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

    /**
     * Returns the estimated robot pose as a {@link RigidTransform2}.
     *
     * The translation is relative to the hub, and the rotation has the same reference as
     * STARTING_ROBOT_ROTATION. Positive rotation is clockwise, negative is counterclockwise.
     *
     * @return The estimated robot pose as a {@link RigidTransform2}.
     */
    public RigidTransform2 getEstimatedPose() {
        double distanceToHub = getDistance();
        Rotation2 robotAngle = gyro.getAdjustmentAngle().rotateBy(gyroAdjustmentAngle);
        Rotation2 hubRobotAngle = robotAngle.rotateBy(Rotation2.fromDegrees(180 - getYaw()));
        Vector2 translation = Vector2.fromAngle(hubRobotAngle).scale(distanceToHub);
        return new RigidTransform2(translation, robotAngle);
    }

    public void setLedOn() {
        limelight.getEntry("ledMode").setValue("1");
    }

    public void setLedOff() {
        limelight.getEntry("ledMode").setValue("0");
    }

}
