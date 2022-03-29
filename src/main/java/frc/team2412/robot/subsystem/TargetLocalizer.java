package frc.team2412.robot.subsystem;

import frc.team2412.robot.Robot;
import frc.team2412.robot.util.GeoConvertor;
import frc.team2412.robot.util.TimeBasedMedianFilter;
import frc.team2412.robot.util.autonomous.AutonomousChooser;

import static frc.team2412.robot.subsystem.TargetLocalizer.LocalizerConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Class that combines sensor data from multiple subsystems. <br>
 *
 * <p>
 * Field-centric pose frame of reference:
 * <ul>
 * <li>Units are inches</li>
 * <li>Origin is right corner of alliance wall</li>
 * <li>Positive X axis points away from alliance wall</li>
 * <li>Positive Y axis points left from alliance wall</li>
 * <li>0 rotation points away from alliance wall</li>
 * <li>Positive rotation is counterclockwise</li>
 * </ul>
 * </p>
 */
public class TargetLocalizer {
    public static class LocalizerConstants {
        // TODO tune these more
        public static final double TURRET_LATERAL_FF = 0, TURRET_ANGULAR_FF = 4, TURRET_DEPTH_FF = 0;
        // Seconds, placeholder duration
        public static final double FILTER_TIME = 1;
        // Angles are in degrees
        // Positive is clockwise
        public static final double STARTING_TURRET_ANGLE = 0;
        // Dimensions are in inches
        public static final double VISION_DEFAULT_DISTANCE = 118;
        // Estimated, negative because limelight is in back of turret
        public static final double TURRET_CENTER_TO_LIMELIGHT_DISTANCE = -7;
        public static final Translation2d ROBOT_CENTRIC_TURRET_CENTER = new Translation2d(3.93, -4);
        public static final Translation2d FIELD_CENTRIC_HUB_CENTER = new Translation2d(27 * 12, 13.5 * 12);
    }

    private final DrivebaseSubsystem drivebaseSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final ShooterVisionSubsystem shooterVisionSubsystem;
    private final TimeBasedMedianFilter distanceFilter;
    private Pose2d startingPose;
    private Rotation2d gyroAdjustmentAngle;

    /**
     * Creates a new {@link TargetLocalizer}.
     * If {@code drivebaseSubsystem} is null, will assume robot is stationary.
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
        resetPose(new Pose2d());
    }

    /**
     * Rests the current pose according to the current auto path.
     *
     * @param autoChooser
     *            The autonomous chooser with the current auto path.
     */
    public void resetPoseFromAutoStartPose(AutonomousChooser autoChooser) {
        resetPose(autoChooser.getStartPose());
    }

    /**
     * Resets the current pose.
     *
     * @param newPose
     *            The new (field-centric) pose.
     */
    public void resetPose(Pose2d newPose) {
        this.startingPose = newPose;
        this.gyroAdjustmentAngle = startingPose.getRotation().minus(getGyroscopeYaw());
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
     * @return The yaw to the hub (0 is straight ahead, positive is clockwise, units are degrees).
     */
    public double getVisionYaw() {
        return shooterVisionSubsystem.getYaw();
    }

    /**
     * Returns the yaw (horizontal angle) to the target position.
     *
     * @return The yaw to the hub plus turret angle bias (0 is straight ahead, positive is clockwise,
     *         units are degrees).
     */
    public double getTargetYaw() {
        // return 0;
        return getVisionYaw() + shooterSubsystem.getTurretAngleBias();
    }

    /**
     * Returns the gyroscope's yaw.
     *
     * @return The gyroscope's yaw without the adjustment angle (positive rotation is counterclockwise).
     */
    public Rotation2d getGyroscopeYaw() {
        return (drivebaseSubsystem != null) ? drivebaseSubsystem.getGyroscopeUnadjustedAngle().unaryMinus()
                : new Rotation2d();
    }

    /**
     * Return the robot's angle relative to the field.
     *
     * @return The robot angle (0 is straight forward from the driver station, positive rotation is
     *         counterclockwise).
     */
    public Rotation2d getFieldCentricRobotAngle() {
        return getGyroscopeYaw().rotateBy(gyroAdjustmentAngle);
    }

    /**
     * Returns the turret's angle relative to the field.
     *
     * @return The turret angle (0 is straight forward from the driver station, positive rotation is
     *         counterclockwise).
     */
    public Rotation2d getFieldCentricTurretAngle() {
        return getFieldCentricRobotAngle().rotateBy(getTurretAngle().unaryMinus());
    }

    /**
     * Return the turret's angle.
     *
     * @return The turret angle (0 is intake side, positive rotation is clockwise).
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
     * Returns the estimated limelight pose according to vision and the gyroscope. <br>
     *
     * <p>
     * If the limelight doesn't have a target, returns {@code Pose2d()}.
     * </p>
     *
     * @return The field-centric estimated limelight pose according to vision and the gyroscope.
     */
    public Pose2d getVisionGyroLimelightPose() {
        if (!hasTarget()) {
            return new Pose2d();
        }
        Rotation2d limelightAngle = getFieldCentricTurretAngle();
        Rotation2d hubAngle = limelightAngle.rotateBy(Rotation2d.fromDegrees(-getVisionYaw()));
        Translation2d limelightToHub = new Translation2d(getDistance(), hubAngle);
        Translation2d translation = FIELD_CENTRIC_HUB_CENTER.minus(limelightToHub);
        return new Pose2d(translation, limelightAngle);
    }

    /**
     * Returns the estimated robot pose according to vision and the gyroscope. <br>
     *
     * <p>
     * If the limelight doesn't have a target, returns {@code Pose2d()}.
     * </p>
     *
     * @return The estimated field-centric robot pose according to vision and the gyroscope.
     */
    public Pose2d getVisionGyroRobotPose() {
        if (!hasTarget()) {
            return new Pose2d();
        }
        Rotation2d robotAngle = getFieldCentricRobotAngle();
        Translation2d limelightTranslation = getVisionGyroLimelightPose().getTranslation();
        Translation2d turretToLimelight = new Translation2d(TURRET_CENTER_TO_LIMELIGHT_DISTANCE,
                getFieldCentricTurretAngle());
        Translation2d robotToTurret = ROBOT_CENTRIC_TURRET_CENTER.rotateBy(new Rotation2d(Math.PI / 2))
                .rotateBy(robotAngle);
        Translation2d translation = limelightTranslation.minus(robotToTurret.plus(turretToLimelight));
        return new Pose2d(translation, robotAngle);
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
