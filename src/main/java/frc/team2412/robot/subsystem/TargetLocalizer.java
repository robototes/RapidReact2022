package frc.team2412.robot.subsystem;

import org.frcteam2910.common.math.Rotation2;

import static frc.team2412.robot.subsystem.TargetLocalizer.LocalizerConstants.*;

public class TargetLocalizer {
    public static class LocalizerConstants {
        public static final double TURRET_OFFSET = 0;
        public static final double TURRET_LATERAL_FF = 0, TURRET_ANGULAR_FF = 0;
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
        return hasTarget() ? shooterVisionSubsystem.getDistance() + shooterSubsystem.getDistanceBias() : 0;
    }

    public double getPitch() {
        return shooterVisionSubsystem.getAdjustedPitch();
    }

    public boolean hasTarget() {
        return shooterVisionSubsystem.hasTarget();
    }

    public double getYaw() {
        return shooterVisionSubsystem.getYaw() + shooterSubsystem.getTurretAngleBias();
    }

    public double getLateralVelocity() {
        return drivebaseSubsystem.getVelocity()
                .rotateBy(Rotation2.fromDegrees(TURRET_OFFSET + shooterSubsystem.getTurretAngle())).x;
    }

    public double getAngularVelocity() {
        return drivebaseSubsystem.getAngularVelocity();
    }

    public double getAdjustedYaw() {
        return getYaw() + getLateralVelocity() * TURRET_LATERAL_FF + getAngularVelocity() * TURRET_ANGULAR_FF;
    }
}
