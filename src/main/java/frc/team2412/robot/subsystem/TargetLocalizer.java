package frc.team2412.robot.subsystem;

import frc.team2412.robot.Robot;
import org.frcteam2910.common.math.Rotation2;

import static frc.team2412.robot.subsystem.TargetLocalizer.LocalizerConstants.*;

public class TargetLocalizer {
    public static class LocalizerConstants {
        public static final double TURRET_OFFSET = 0;
        // TODO tune these more
        public static final double TURRET_LATERAL_FF = 4.8, TURRET_ANGULAR_FF = 4.8, TURRET_DEPTH_FF = 0,
                LATERAL_DEPTH_COMPENSATION = 0;
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

    public double getAdjustedDistance() {
        return getDistance() + distanceAdjustment();
    }

    public double distanceAdjustment() {
        return (getDepthVelocity() * TURRET_DEPTH_FF) / getVoltage();
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

    public double getLateralVelocity() {
        return drivebaseSubsystem.getVelocity()
                .rotateBy(Rotation2.fromDegrees(TURRET_OFFSET + shooterSubsystem.getTurretAngle())).x;
    }

    public double getDepthVelocity() {
        return drivebaseSubsystem.getVelocity()
                .rotateBy(Rotation2.fromDegrees(TURRET_OFFSET + shooterSubsystem.getTurretAngle())).y;
    }

    public double getAngularVelocity() {
        System.out.println(drivebaseSubsystem.getAngularVelocity());
        return drivebaseSubsystem.getAngularVelocity();
    }

    public double yawAdjustment() {
        return (getLateralVelocity() * TURRET_LATERAL_FF + getAngularVelocity() * TURRET_ANGULAR_FF) / getVoltage()
                + getDistance() * LATERAL_DEPTH_COMPENSATION;
    }

    public double getVoltage() {
        // return 12;
        return Robot.getInstance().hardware.PDP.getVoltage();
    }
}
