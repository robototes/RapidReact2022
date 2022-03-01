package frc.team2412.robot;

import static frc.team2412.robot.Subsystems.SubsystemConstants.CLIMB_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.DRIVE_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.INDEX_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.INTAKE_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.MONITOR_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.SHOOTER_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.SHOOTER_VISION_ENABLED;

import frc.team2412.robot.subsystem.ClimbSubsystem;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;
import frc.team2412.robot.subsystem.MonitoringSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.ShooterVisionSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Subsystems implements Loggable {
    public static class SubsystemConstants {
        public static final boolean CLIMB_ENABLED = false;
        public static final boolean DRIVE_ENABLED = true;
        public static final boolean DRIVER_VIS_ENABLED = false;
        public static final boolean SHOOTER_VISION_ENABLED = true;
        public static final boolean INDEX_ENABLED = true;
        public static final boolean INTAKE_ENABLED = true;
        public static final boolean SHOOTER_ENABLED = true;
        public static final boolean SHOOTER_TESTING = false;
        public static final boolean MONITOR_ENABLED = true;
    }

    public final Hardware hardware;

    public ClimbSubsystem climbSubsystem;

    public DrivebaseSubsystem drivebaseSubsystem;

    public ShooterVisionSubsystem shooterVisionSubsystem;

    public IndexSubsystem indexSubsystem;

    @Log(tabName = "IntakeSubsystem")
    public IntakeSubsystem intakeSubsystem;

    public ShooterSubsystem shooterSubsystem;

    public MonitoringSubsystem monitoringSubsystem;

    public Subsystems(Hardware h) {
        hardware = h;

        if (CLIMB_ENABLED)
            climbSubsystem = new ClimbSubsystem(hardware.climbMotorFixed, hardware.climbMotorDynamic,
                    hardware.climbAngle);
        if (DRIVE_ENABLED)
            drivebaseSubsystem = new DrivebaseSubsystem(hardware.frontLeftModule, hardware.frontRightModule,
                    hardware.backLeftModule, hardware.backRightModule, hardware.pigeon,
                    Hardware.HardwareConstants.MODULE_MAX_VELOCITY_METERS_PER_SEC);
        if (SHOOTER_VISION_ENABLED)
            shooterVisionSubsystem = new ShooterVisionSubsystem();
        if (INTAKE_ENABLED)
            intakeSubsystem = new IntakeSubsystem(hardware.intakeMotor, hardware.intakeSolenoid);
        if (SHOOTER_ENABLED)
            shooterSubsystem = new ShooterSubsystem(hardware.flywheelMotor1, hardware.flywheelMotor2,
                    hardware.turretMotor, hardware.hoodMotor);
        if (INDEX_ENABLED) {
            indexSubsystem = new IndexSubsystem(hardware.ingestIndexMotor, hardware.feederIndexMotor,
                    hardware.ingestProximity, hardware.feederProximity, hardware.ingestBlueColor,
                    hardware.ingestRedColor, hardware.feederBlueColor, hardware.feederRedColor);
            // indexSubsystem.setDefaultCommand(
            // new IntakeBitmapCommand(intakeSubsystem, indexSubsystem, shooterSubsystem,
            // shooterVisionSubsystem));
        }
        if (MONITOR_ENABLED) {
            monitoringSubsystem = new MonitoringSubsystem(hardware.powerDistributionPanel);
        }
    }
}
