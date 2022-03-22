package frc.team2412.robot.subsystem;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import frc.team2412.robot.Robot;

import static frc.team2412.robot.subsystem.TargetLocalizer.LocalizerConstants.*;

public class TargetLocalizer {
    public static class LocalizerConstants {
        // TODO tune these more
        public static final double TURRET_LATERAL_FF = 0.1, TURRET_ANGULAR_FF = 10, TURRET_DEPTH_FF = 0.1;
        // Angles are in degrees
        public static final double STARTING_TURRET_ANGLE = 0;
        // Dimensions are in inches
        // Estimated, negative because limelight is in back of turret
        public static final double LIMELIGHT_TO_TURRET_CENTER_DISTANCE = -7;
        public static final Vector2 ROBOT_CENTER_TO_TURRET_CENTER = new Vector2(3.93, -4);
    }

    private final DrivebaseSubsystem drivebaseSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final ShooterVisionSubsystem shooterVisionSubsystem;
    private final Rotation2 gyroAdjustmentAngle;
    private final RigidTransform2 startingPose;

    public TargetLocalizer(DrivebaseSubsystem drivebase, ShooterSubsystem shooter, ShooterVisionSubsystem vision) {
        drivebaseSubsystem = drivebase;
        shooterSubsystem = shooter;
        shooterVisionSubsystem = vision;
        // TODO Handle different starting positions
        // Also don't forget to convert reference to hub-centric if necessary
        startingPose = new RigidTransform2(new Vector2(5 * 12, 5 * 12), Rotation2.ZERO);
        gyroAdjustmentAngle = startingPose.rotation.rotateBy(drivebase.getGyroscopeUnadjustedAngle().inverse());
    }

    public double getDistance() {
        return hasTarget() ? shooterVisionSubsystem.getDistance() + shooterSubsystem.getDistanceBias() : 0;
    }

    public double getAdjustedDistance() {
        return getDistance() + distanceAdjustment();
    }

    /**
     * very basic feedforward math to adjust the depth depending on the distance you are moving away
     * from target
     *
     * @return adjustment
     */
    public double distanceAdjustment() {
        return (getDepthVelocity() * getDistance() * TURRET_DEPTH_FF) / getVoltage();
    }

    public double getPitch() {
        return shooterVisionSubsystem.getAdjustedPitch();
    }

    public boolean hasTarget() {
        return shooterVisionSubsystem.hasTarget();
    }

    /**
     * Returns the yaw (horizontal angle) to the hub.
     *
     * @return The yaw (horizontal angle) to the hub (0 is straight ahead, positive is clockwise, units
     *         are degrees).
     */
    public double getVisionYaw() {
        return shooterVisionSubsystem.getYaw();
    }

    /**
     * Returns the yaw (horizontal angle) to the target position.
     *
     * @return The yaw to the hub plus any turret angle bias, see {@link #getVisionYaw()} for more
     *         details.
     */
    public double getTargetYaw() {
        // return 0;
        return getVisionYaw() + shooterSubsystem.getTurretAngleBias();
    }

    /**
     * Return the robot's angle relative to the field.
     *
     * @return The robot angle relative to the field (same basis as starting pose).
     */
    public Rotation2 getFieldCentricRobotAngle() {
        return drivebaseSubsystem.getGyroscopeUnadjustedAngle().rotateBy(gyroAdjustmentAngle);
    }

    /**
     * Return the turret's angle.
     *
     * @return The turret angle, 0 is intake side, positive is clockwise.
     */
    public Rotation2 getTurretAngle() {
        return Rotation2.fromDegrees(shooterSubsystem.getTurretAngle() + STARTING_TURRET_ANGLE);
    }

    /**
     * unit vector component of chassis velocity perpendicular to the turret output
     *
     * @return that
     */
    public double getLateralVelocity() {
        // might need to do inverse
        return drivebaseSubsystem.getVelocity().rotateBy(getTurretAngle()).x;
    }

    /**
     * unit vector component of chassis velocity parallel to the turret output
     *
     * @return that
     */
    public double getDepthVelocity() {
        // might need to do inverse
        return drivebaseSubsystem.getVelocity().rotateBy(getTurretAngle()).y;
    }

    public double getAngularVelocity() {
        return drivebaseSubsystem.getAngularVelocity();
    }

    /**
     * feedforward math for turret angle feedforward
     * multiply the lateral velocity by distance.
     * This is to compensate for a longer time of flight the farther away you are
     * and it is not perfect but it should work.
     * angular velocity is to help the turret keep heading when the robot itself is turning
     *
     * @return adjustment
     */
    public double yawAdjustment() {
        return (getLateralVelocity() * getDistance() * TURRET_LATERAL_FF + getAngularVelocity() * TURRET_ANGULAR_FF)
                / getVoltage();
    }

    // TODO Validate references

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
        Rotation2 fieldCentricLimelightAngle = getFieldCentricRobotAngle().rotateBy(getTurretAngle());
        Rotation2 fieldCentricLimelightToHubAngle = fieldCentricLimelightAngle
                .rotateBy(Rotation2.fromDegrees(getVisionYaw()));
        Rotation2 fieldCentricHubToLimelightAngle = fieldCentricLimelightToHubAngle
                .rotateBy(Rotation2.fromDegrees(180));
        Vector2 translation = Vector2.fromAngle(fieldCentricHubToLimelightAngle).scale(distanceToHub);
        return new RigidTransform2(translation, fieldCentricLimelightAngle);
    }

    /**
     * Returns the estimated robot pose.
     *
     * The translation (inches) is relative to the hub, and the rotation is relative to straight
     * forward
     * from the drive station. Positive rotation is clockwise. If the limelight doesn't have a target,
     * returns {@link RigidTransform2#ZERO}.
     *
     * @return The estimated robot pose.
     */
    public RigidTransform2 getEstimatedRobotPose() {
        if (!hasTarget()) {
            return RigidTransform2.ZERO;
        }
        Vector2 hubToLimelight = getEstimatedLimelightPose().translation;
        Vector2 turretToLimelight = Vector2.fromAngle(getTurretAngle()).scale(LIMELIGHT_TO_TURRET_CENTER_DISTANCE);
        Vector2 robotToLimelight = ROBOT_CENTER_TO_TURRET_CENTER.add(turretToLimelight);
        Vector2 hubToRobot = hubToLimelight.subtract(robotToLimelight);
        return new RigidTransform2(hubToRobot, getFieldCentricRobotAngle());
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
        return getEstimatedRobotPose().transformBy(startingPose.inverse());
    }

    public void limelightOn() {
        shooterVisionSubsystem.setLedOn();
    }

    public void limelightOff() {
        shooterVisionSubsystem.setLedOff();
    }

    public double getVoltage() {
        return Robot.getInstance().getVoltage();
    }
}
