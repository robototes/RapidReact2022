package frc.team2412.robot.subsystem;

import org.frcteam2910.common.math.Rotation2;

import frc.team2412.robot.Robot;

import static frc.team2412.robot.subsystem.TargetLocalizer.LocalizerConstants.*;

public class TargetLocalizer {
    public static class LocalizerConstants {
        public static final double TURRET_OFFSET = 0;
        // TODO tune these more
        public static final double TURRET_LATERAL_FF = 0, TURRET_ANGULAR_FF = 5, TURRET_DEPTH_FF = 0;
    }

    private final DrivebaseSubsystem drivebaseSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final ShooterVisionSubsystem shooterVisionSubsystem;

    public TargetLocalizer(DrivebaseSubsystem drivebase, ShooterSubsystem shooter, ShooterVisionSubsystem vision) {
        drivebaseSubsystem = drivebase;
        shooterSubsystem = shooter;
        shooterVisionSubsystem = vision;
    }

    public double getDistance() {
        return hasTarget() ? shooterVisionSubsystem.getDistance() + shooterSubsystem.getDistanceBias()
                : ShooterVisionSubsystem.ShooterVisionConstants.HUB_RADIUS;
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
        // might need to do inverse
        return drivebaseSubsystem.getVelocity()
                .rotateBy(Rotation2.fromDegrees(TURRET_OFFSET + shooterSubsystem.getTurretAngle())).x;
    }

    /**
     * unit vector component of chassis velocity parallel to the turret output
     *
     * @return that
     */
    public double getDepthVelocity() {
        // might need to do inverse
        return drivebaseSubsystem.getVelocity()
                .rotateBy(Rotation2.fromDegrees(TURRET_OFFSET + shooterSubsystem.getTurretAngle())).y;
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
        return (Math.toDegrees(Math.asin(getLateralVelocity() / getDistance() * TURRET_LATERAL_FF))
                + getAngularVelocity() * TURRET_ANGULAR_FF)
                / getVoltage();
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
