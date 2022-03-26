package frc.team2412.robot.subsystem;

import frc.team2412.robot.Robot;
import frc.team2412.robot.util.GeoConvertor;
import frc.team2412.robot.util.TimeBasedMedianFilter;

import static frc.team2412.robot.subsystem.TargetLocalizer.LocalizerConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class TargetLocalizer {
    public static class LocalizerConstants {
        // TODO tune these more
        public static final double TURRET_LATERAL_FF = 0, TURRET_ANGULAR_FF = 4, TURRET_DEPTH_FF = 0;
        // Seconds, placeholder duration
        public static final double FILTER_TIME = 1;
        // Angles are in degrees
        public static final double STARTING_TURRET_ANGLE = 0;
        // Dimensions are in meters
        public static final double VISION_DEFAULT_DISTANCE = Units.inchesToMeters(118);
        // Estimated, negative because limelight is in back of turret
        public static final double LIMELIGHT_TO_TURRET_CENTER_DISTANCE = Units.inchesToMeters(-7);
        public static final Translation2d ROBOT_CENTRIC_TURRET_CENTER = new Translation2d(Units.inchesToMeters(3.93),
                Units.inchesToMeters(-4));
    }

    private final DrivebaseSubsystem drivebaseSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final ShooterVisionSubsystem shooterVisionSubsystem;
    private final TimeBasedMedianFilter distanceFilter;
    private final Rotation2d gyroAdjustmentAngle;
    private final Pose2d startingPose;

    /**
     * Creates a new {@link TargetLocalizer}.
     * If {@code drivebase} is null, will assume robot is stationary.
     *
     * @param drivebaseSubsystem
     *            The drivebase subsystem.
     * @param shooterSubsystem
     *            The shooter subsystem.
     * @param visionSubsystem
     *            The vision subsystem.
     */
    public TargetLocalizer(DrivebaseSubsystem drivebaseSubsystem, ShooterSubsystem shooterSubsystem,
            ShooterVisionSubsystem visionSubsystem) {
        this.drivebaseSubsystem = drivebaseSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.shooterVisionSubsystem = visionSubsystem;
        this.distanceFilter = new TimeBasedMedianFilter(FILTER_TIME);
        // TODO Handle different starting positions
        // Also don't forget to convert reference to hub-centric if necessary
        this.startingPose = new Pose2d(Units.feetToMeters(5), Units.feetToMeters(5), new Rotation2d());
        this.gyroAdjustmentAngle = startingPose.getRotation().minus(drivebaseSubsystem.getGyroscopeUnadjustedAngle());
    }

    public double getDistance() {
        return hasTarget()
                ? distanceFilter.calculate(shooterVisionSubsystem.getDistance() + shooterSubsystem.getDistanceBias())
                : VISION_DEFAULT_DISTANCE;
    }

    public double getAdjustedDistance() {
        return getDistance() + distanceAdjustment();
    }

    /**
     * very basic feedforward math to adjust the depth depending on the distance you
     * are moving away
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
     * @return The yaw to the hub (0 is straight ahead, positive is clockwise, units
     *         are degrees).
     */
    public double getVisionYaw() {
        return shooterVisionSubsystem.getYaw();
    }

    /**
     * Returns the yaw (horizontal angle) to the target position.
     *
     * @return The yaw to the hub plus turret angle bias (0 is straight ahead,
     *         positive is clockwise,
     *         units are degrees).
     */
    public double getTargetYaw() {
        // return 0;
        return getVisionYaw() + shooterSubsystem.getTurretAngleBias();
    }

    /**
     * Return the robot's angle relative to the field.
     *
     * @return The robot angle (0 is straight forward from the driver station,
     *         positive rotation is
     *         clockwise).
     */
    public Rotation2d getFieldCentricRobotAngle() {
        return drivebaseSubsystem.getGyroscopeUnadjustedAngle().rotateBy(gyroAdjustmentAngle);
    }

    /**
     * Return the turret's angle.
     *
     * @return The turret angle (0 is intake side, positive is clockwise).
     */
    public Rotation2d getTurretAngle() {
        return Rotation2d.fromDegrees(shooterSubsystem.getTurretAngle() + STARTING_TURRET_ANGLE);
    }

    public Translation2d getTurretRelativeVelocity() {
        return (drivebaseSubsystem != null)
                // might need to do inverse
                ? GeoConvertor.vector2ToTranslation2d(drivebaseSubsystem.getVelocity()).rotateBy(getTurretAngle())
                : new Translation2d();
    }

    /**
     * unit vector component of chassis velocity perpendicular to the turret output
     *
     * @return that
     */
    public double getLateralVelocity() {
        return getTurretRelativeVelocity().getX();
    }

    /**
     * unit vector component of chassis velocity parallel to the turret output
     *
     * @return that
     */
    public double getDepthVelocity() {
        return getTurretRelativeVelocity().getX();
    }

    public double getAngularVelocity() {
        return (drivebaseSubsystem != null) ? drivebaseSubsystem.getAngularVelocity() : 0;
    }

    /**
     * feedforward math for turret angle feedforward
     * multiply the lateral velocity by distance.
     * This is to compensate for a longer time of flight the farther away you are
     * and it is not perfect but it should work.
     * angular velocity is to help the turret keep heading when the robot itself is
     * turning
     *
     * @return adjustment
     */
    public double yawAdjustment() {
        return (getDistance() != 0 && getDistance() > getLateralVelocity()
                ? Math.toDegrees(Math.asin(getLateralVelocity() / getDistance() * TURRET_LATERAL_FF))
                : 0) + (getAngularVelocity() * TURRET_ANGULAR_FF)
                        / getVoltage();
    }

    /**
     * Returns the estimated limelight pose according to vision and the gyroscope.
     *
     * The translation (inches) is relative to the hub, and the rotation is relative to straight forward
     * from the driver station (Positive rotation is clockwise). If the limelight doesn't have a target,
     * returns {@code Pose2d()}.
     *
     * For example, returning {@code Pose2d(1, -2, Rotation2d.fromDegrees(20))} means that, looking out
     * from the driver station, the limelight is one meter to the right of and two meters in front of
     * the center of the hub and is pointing 20 degrees to the right (clockwise).
     *
     * @return The estimated limelight pose according to vision and the gyroscope.
     */
    public Pose2d getVisionGyroLimelightPose() {
        if (!hasTarget()) {
            return new Pose2d();
        }
        Rotation2d fieldCentricLimelightAngle = getFieldCentricRobotAngle().rotateBy(getTurretAngle());
        Rotation2d fieldCentricLimelightToHubAngle = fieldCentricLimelightAngle
                .rotateBy(Rotation2d.fromDegrees(getVisionYaw()));
        Rotation2d fieldCentricHubToLimelightAngle = fieldCentricLimelightToHubAngle
                .rotateBy(Rotation2d.fromDegrees(180));
        Translation2d forwardXTranslation = new Translation2d(getDistance(), fieldCentricHubToLimelightAngle);
        Translation2d forwardYTranslation = forwardXTranslation.rotateBy(Rotation2d.fromDegrees(90));
        return new Pose2d(forwardYTranslation, fieldCentricLimelightAngle);
    }

    /**
     * Returns the estimated robot pose according to vision and the gyroscope.
     *
     * The translation (inches) is relative to the hub, and the rotation is relative to straight forward
     * from the drive station (Positive rotation is clockwise). If the limelight doesn't have a target,
     * returns {@code Pose2d()}.
     *
     * For example, returning {@code Pose2d(1, -2, Rotation.fromDegrees(20))} means that, looking from
     * the driver station, the center of the robot is one meter to the right of and two meters in front
     * of the center of the hub and is pointing 20 degrees to the right (clockwise).
     *
     * @return The estimated robot pose according to vision and the gyroscope.
     */
    public Pose2d getVisionGyroRobotPose() {
        if (!hasTarget()) {
            return new Pose2d();
        }
        Translation2d fieldCentricHubToLimelight = getVisionGyroLimelightPose().getTranslation();
        Translation2d robotCentricTurretToLimelight = new Translation2d(LIMELIGHT_TO_TURRET_CENTER_DISTANCE,
                getTurretAngle());
        Translation2d robotCentricLimelightPosition = ROBOT_CENTRIC_TURRET_CENTER.plus(robotCentricTurretToLimelight);
        Translation2d fieldCentricRobotToLimelight = robotCentricLimelightPosition.rotateBy(getFieldCentricRobotAngle())
                .rotateBy(Rotation2d.fromDegrees(90));
        Translation2d hubToRobot = fieldCentricHubToLimelight.minus(fieldCentricRobotToLimelight);
        return new Pose2d(hubToRobot, getFieldCentricRobotAngle());
    }

    /**
     * Returns the estimated robot pose relative to the start according to vision and the gyroscope.
     *
     * The translation (inches) is from the starting position, and the rotation is relative to the
     * starting rotation (Positive is clockwise). If the limelight doesn't have a target, returns
     * {@code Pose2d()}.
     *
     * For example, returning {@code Pose2d(1, -2, Rotation2d.fromDegrees(20))} means that, looking from
     * the driver station, the robot moved one foot to the right and two feet closer, and rotated 20
     * degrees clockwise.
     *
     * @return The estimated robot pose relative to the start according to vision and the gyroscope.
     */
    public Pose2d getVisionGyroRobotPoseRelativeToStart() {
        return hasTarget() ? getVisionGyroRobotPose().transformBy(startingPose.minus(new Pose2d())) : new Pose2d();
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

    public boolean upToSpeed() {
        return shooterSubsystem.upToSpeed();
    }
}
