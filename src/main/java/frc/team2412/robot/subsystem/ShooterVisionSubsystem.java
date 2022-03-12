package frc.team2412.robot.subsystem;

import static frc.team2412.robot.subsystem.ShooterVisionSubsystem.ShooterVisionConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Hardware;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ShooterVisionSubsystem extends SubsystemBase implements Loggable {
    public static class ShooterVisionConstants {
        // Dimensions are in inches
        public static final double LIMELIGHT_HEIGHT_OFFSET = 39;
        public static final double RIM_HEIGHT = 8 * 12 + 8;
        public static final double HEIGHT_TO_RIM = RIM_HEIGHT - LIMELIGHT_HEIGHT_OFFSET;
        public static final double HUB_RADIUS = 4 * 12 / 2;
        // TODO Handle different starting positions
        public static final Vector2 STARTING_ROBOT_POSITION = new Vector2(5 * 12, 5 * 12);
        public static final double LIMELIGHT_TO_TURRET_CENTER_DISTANCE = -7; // Estimated, negative because limelight is
                                                                                // in back of turret
        public static final Vector2 ROBOT_CENTER_TO_TURRET_CENTER = new Vector2(3.93, -4);
        // Rotation2 can be specified by degrees or radians
        // STARTING_ROBOT_ROTATION of 0 means straight forward from the driver station
        public static final Rotation2 STARTING_ROBOT_ROTATION = Rotation2.ZERO; // Placeholder
        // Angles are in degrees
        public static final double LIMELIGHT_ANGLE_OFFSET = Math
                .toDegrees(Math.atan2(HEIGHT_TO_RIM, 30 * 12 - HUB_RADIUS));
        public static final double STARTING_TURRET_ANGLE = 90;
    }

    public NetworkTable limelight;

    private final Gyroscope gyro;

    private final Rotation2 gyroAdjustmentAngle;

    private final Supplier<Rotation2> turretAngleSupplier;

    public ShooterVisionSubsystem(Gyroscope gyro, DoubleSupplier turretAngleSupplier) {
        this.limelight = NetworkTableInstance.getDefault().getTable(Hardware.HardwareConstants.LIMELIGHT);
        this.gyro = gyro;
        this.gyroAdjustmentAngle = STARTING_ROBOT_ROTATION.rotateBy(gyro.getUnadjustedAngle().inverse());
        this.turretAngleSupplier = () -> Rotation2
                .fromDegrees(turretAngleSupplier.getAsDouble() + STARTING_TURRET_ANGLE);
    }

    @Override
    public void periodic() {

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

    /**
     * Returns the estimated limelight pose.
     *
     * The translation (inches) is relative to the hub, and the rotation is relative to straight forward
     * from the driver station. Positive rotation is clockwise. If the limelight doesn't have a target,
     * returns {@link RigidTransform2#ZERO}.
     *
     * @return The estimated limelight pose.
     */
    public RigidTransform2 getEstimatedLimelightPose() {
        if (!hasTarget()) {
            return RigidTransform2.ZERO;
        }
        double distanceToHub = getDistance();
        Rotation2 fieldCentricRobotAngle = gyro.getUnadjustedAngle().rotateBy(gyroAdjustmentAngle);
        Rotation2 fieldCentricLimelightAngle = fieldCentricRobotAngle.rotateBy(turretAngleSupplier.get());
        Rotation2 fieldCentricLimelightToHubAngle = fieldCentricLimelightAngle
                .rotateBy(Rotation2.fromDegrees(getYaw()));
        Rotation2 fieldCentricHubToLimelightAngle = fieldCentricLimelightToHubAngle
                .rotateBy(Rotation2.fromDegrees(180));
        Vector2 translation = Vector2.fromAngle(fieldCentricHubToLimelightAngle).scale(distanceToHub);
        return new RigidTransform2(translation, fieldCentricLimelightAngle);
    }

    /**
     * Returns the estimated robot pose.
     *
     * The translation (inches) is relative to the hub, and the rotation is relative to straight forward
     * from the drive station. Positive rotation is clockwise. If the limelight doesn't have a target,
     * returns {@link RigidTransform2#ZERO}.
     *
     * @return The estimated robot pose.
     */
    public RigidTransform2 getEstimatedRobotPose() {
        if (!hasTarget()) {
            return RigidTransform2.ZERO;
        }
        Rotation2 fieldCentricRobotAngle = gyro.getUnadjustedAngle().rotateBy(gyroAdjustmentAngle);
        Vector2 hubToLimelight = getEstimatedLimelightPose().translation;
        Vector2 turretToLimelight = Vector2.fromAngle(turretAngleSupplier.get())
                .scale(LIMELIGHT_TO_TURRET_CENTER_DISTANCE);
        Vector2 robotToLimelight = ROBOT_CENTER_TO_TURRET_CENTER.add(turretToLimelight);
        Vector2 hubToRobot = hubToLimelight.subtract(robotToLimelight);
        return new RigidTransform2(hubToRobot, fieldCentricRobotAngle);
    }

    /**
     * Returns the estimated robot pose relative to the start.
     *
     * The translation (inches) is relative to the starting position, and the rotation is relative to
     * the starting rotation. Positive is clockwise. If the limelight doesn't have a target, returns
     * {@link RigidTransform2#ZERO}.
     *
     * @return The estimated robot pose relative to the start.
     */
    public RigidTransform2 getEstimatedRobotPoseRelativeToStart() {
        if (!hasTarget()) {
            return RigidTransform2.ZERO;
        }
        RigidTransform2 hubRelativePose = getEstimatedRobotPose();
        Vector2 translationFromStart = hubRelativePose.translation.subtract(STARTING_ROBOT_POSITION);
        Rotation2 rotationFromStart = hubRelativePose.rotation.rotateBy(STARTING_ROBOT_ROTATION.inverse());
        return new RigidTransform2(translationFromStart, rotationFromStart);
    }

    public void setLedOn() {
        limelight.getEntry("ledMode").setValue("1");
    }

    public void setLedOff() {
        limelight.getEntry("ledMode").setValue("0");
    }

}
