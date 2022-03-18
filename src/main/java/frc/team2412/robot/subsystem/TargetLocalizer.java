package frc.team2412.robot.subsystem;

import org.frcteam2910.common.math.Rotation2;

import static frc.team2412.robot.subsystem.TargetLocalizer.LocalizerConstants.*;

public class TargetLocalizer {
    public static class LocalizerConstants {
        public static final double TURRET_OFFSET = 0;
        // TODO tune these more
        public static final double TURRET_LATERAL_FF = 0.1, TURRET_ANGULAR_FF = 5, TURRET_DEPTH_FF = 0.1;
    }

    private final DrivebaseSubsystem drivebaseSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final ShooterVisionSubsystem shooterVisionSubsystem;

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

    public double getYaw() {
        // return 0;
        return shooterVisionSubsystem.getYaw() + shooterSubsystem.getTurretAngleBias();
    }

    /**
     * unit vector component of chassis velocity perpendicular to the turret output
     *
     * @return that
     */
    public double getLateralVelocity() {
        return (drivebaseSubsystem != null)
                // might need to do inverse
                ? drivebaseSubsystem.getVelocity()
                        .rotateBy(Rotation2.fromDegrees(TURRET_OFFSET + shooterSubsystem.getTurretAngle())).x
                : 0;
    }

    /**
     * unit vector component of chassis velocity parallel to the turret output
     *
     * @return that
     */
    public double getDepthVelocity() {
        return (drivebaseSubsystem != null)
                // might need to do inverse
                ? drivebaseSubsystem.getVelocity()
                        .rotateBy(Rotation2.fromDegrees(TURRET_OFFSET + shooterSubsystem.getTurretAngle())).y
                : 0;
    }

    public double getAngularVelocity() {
        return (drivebaseSubsystem != null) ? drivebaseSubsystem.getAngularVelocity() : 0;
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

    public double getVoltage() {
        return 12;
        // WHERE THE PDP GO AAAAAAAAA
        // return Robot.getInstance().PDP.getVoltage();
    }

    public void limelightOn() {
        shooterVisionSubsystem.setLedOn();
    }

    public void limelightOff() {
        shooterVisionSubsystem.setLedOff();
    }
}
