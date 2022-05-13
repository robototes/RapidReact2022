package frc.team2412.robot.subsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import frc.team2412.robot.Robot;
import frc.team2412.robot.util.GeoConvertor;
import frc.team2412.robot.util.VisionUtil;
import frc.team2412.robot.util.autonomous.AutonomousChooser;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import static frc.team2412.robot.subsystem.TargetLocalizer.LocalizerConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Class that combines sensor data from multiple subsystems. <br>
 *
 * <p>
 * See {@link VisionUtil} for the field coordinate system.
 * </p>
 */
public class TargetLocalizer implements Loggable {
    public static class LocalizerConstants {
        // TODO tune these more
        /*
         * Order to tune:
         * turret angular
         * depth FF
         * lateral FF
         * lateral factor
         */
        public static final double TURRET_LATERAL_FF = 0.9, TURRET_ANGULAR_FF = 0.25, TURRET_DEPTH_FF = 0.006, // 0.145
                TURRET_LATERAL_FACTOR = 0;
        public static final double LATERAL_MAX = 80;
        // Seconds, placeholder duration
        public static final double FILTER_TIME = 0.1;
        // Dimensions are in inches
        // Estimated, negative because limelight is in back of turret
        public static final double TURRET_CENTER_TO_LIMELIGHT_DISTANCE = -7;
        // +X axis is forward
        public static final Translation2d ROBOT_CENTRIC_TURRET_CENTER = new Translation2d(-4, -3.93);
        public static final Translation2d FIELD_CENTRIC_HUB_CENTER = new Translation2d(27 * 12, 13.5 * 12);
    }

    @Log.Exclude
    @Config.Exclude
    private final DrivebaseSubsystem drivebaseSubsystem;

    @Log.Exclude
    @Config.Exclude
    private final ShooterSubsystem shooterSubsystem;

    @Log.Exclude
    @Config.Exclude
    private final ShooterVisionSubsystem shooterVisionSubsystem;

    private final LinearFilter distanceFilter;
    private final LinearFilter yawPass;
    private Rotation2d gyroAdjustmentAngle;

    private double turretLateralFF = TURRET_LATERAL_FF;
    private double turretDepthFF = TURRET_DEPTH_FF;
    private double turretAngularFF = TURRET_ANGULAR_FF;
    private double turretDepthLateralFactor = TURRET_LATERAL_FACTOR;

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
        this.distanceFilter = LinearFilter.movingAverage(10);
        this.yawPass = LinearFilter.movingAverage(5);
        resetPose(new Pose2d());
    }

    /**
     * Resets the current pose according to the current auto path.
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
        this.gyroAdjustmentAngle = newPose.getRotation().minus(getGyroscopeYaw());
    }

    public double getDistance() {
        double distance = shooterVisionSubsystem.getDistance() + shooterSubsystem.getDistanceBias();
        return hasTarget() ? distanceFilter.calculate(distance) : distance;
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
        if (getDepthVelocity() < 0.1) {
            return 0;
        }
        return (getDepthVelocity() * Math.sqrt(
                getDistance() * getDistance()
                        + ((getLateralVelocity() * getLateralVelocity()) * turretDepthLateralFactor))
                * turretDepthFF);
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
        return yawPass.calculate(shooterVisionSubsystem.getYaw());
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
     * Return the turret's angle where clockwise rotation is positive.
     *
     * @return The turret angle (0 is intake side, positive rotation is clockwise).
     */
    public Rotation2d getCWPositiveTurretAngle() {
        return Rotation2d.fromDegrees(shooterSubsystem.getTurretAngle());
    }

    /**
     * Returns the turret's angle where counterclockwise is positive.
     *
     * @return The turret angle (0 is intake side, positive rotation is counterclockwise).
     */
    public Rotation2d getTurretAngle() {
        return getCWPositiveTurretAngle().unaryMinus();
    }

    public Translation2d getTurretRelativeVelocity() {
        return (drivebaseSubsystem != null)
                // might need to do inverse
                ? GeoConvertor.vector2ToTranslation2d(drivebaseSubsystem.getVelocity())
                        .rotateBy(getCWPositiveTurretAngle())
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
        return getTurretRelativeVelocity().getY();
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
        double lateralAdjustment = 0;
        double lateralVelocity = getLateralVelocity();

        if (getDistance() != 0 && getDistance() > lateralVelocity) {
            double adjustedLateralVelocity = MathUtil.clamp(lateralVelocity, -LATERAL_MAX, LATERAL_MAX);
            lateralAdjustment = Math.toDegrees(Math.asin(adjustedLateralVelocity / getDistance() * turretLateralFF));
        }

        double angularAdjustment = getAngularVelocity() * turretAngularFF;
        return lateralAdjustment + angularAdjustment;
    }

    /**
     * Returns the estimated robot pose according to vision and the gyroscope.
     *
     * <p>
     * If the limelight doesn't have a target, returns {@code new Pose2d()}.
     *
     * @return The estimated field-centric robot pose according to vision and the gyroscope.
     */
    public Pose2d estimateRobotPoseFromVisionGyro() {
        if (!hasTarget()) {
            return new Pose2d();
        }
        double targetDistance = getDistance();
        Rotation2d targetYaw = Rotation2d.fromDegrees(-getVisionYaw());
        Translation2d targetPosition = FIELD_CENTRIC_HUB_CENTER;
        Rotation2d robotAngle = getFieldCentricRobotAngle();
        Rotation2d robotToCameraAngle = getTurretAngle();
        Translation2d robotCentricCameraPosition = ROBOT_CENTRIC_TURRET_CENTER
                .plus(new Translation2d(TURRET_CENTER_TO_LIMELIGHT_DISTANCE, robotToCameraAngle));
        return VisionUtil.estimateRobotPose(targetPosition, targetDistance, targetYaw, robotAngle,
                robotToCameraAngle, robotCentricCameraPosition);
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

    private boolean ignoreUpToSpeed = false;

    public boolean upToSpeed() {
        return ignoreUpToSpeed ? true : shooterSubsystem.upToSpeed();
    }

    @Config(name = "ignore up to speed")
    public void ignoreUpToSpeed(boolean ignore) {
        ignoreUpToSpeed = ignore;
    }

    @Config(name = "Depth FF", defaultValueNumeric = TURRET_DEPTH_FF)
    public void setFDepth(double f) {
        turretDepthFF = f;
    }

    @Config(name = "Lateral FF", defaultValueNumeric = TURRET_LATERAL_FF)
    public void setFLateral(double f) {
        turretLateralFF = f;
    }

    @Config(name = "Angular FF", defaultValueNumeric = TURRET_ANGULAR_FF)
    public void setFAngular(double f) {
        turretAngularFF = f;
    }

    @Config(name = "later depth FF", defaultValueNumeric = TURRET_LATERAL_FACTOR)
    public void setFLaterDepth(double f) {
        turretDepthLateralFactor = f;
    }

}
