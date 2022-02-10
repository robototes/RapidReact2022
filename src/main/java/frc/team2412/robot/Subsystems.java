package frc.team2412.robot;

import frc.team2412.robot.subsystem.*;

import static frc.team2412.robot.Subsystems.SubsystemConstants.*;

public class Subsystems {
    public static class SubsystemConstants {
        public static final boolean CLIMB_ENABLED = false;
        public static final boolean DRIVE_ENABLED = true;
        public static final boolean DRIVER_VIS_ENABLED = false;
        public static final boolean GOAL_VIS_ENABLED = false;
        public static final boolean INDEX_ENABLED = false;
        public static final boolean INTAKE_ENABLED = false;
        public static final boolean SHOOTER_ENABLED = false;
        public static final boolean I2C_MUX_ENABLED = false;
        public static final boolean MONITOR_ENABLED = false;
    }

    public final Hardware hardware;

    public ClimbSubsystem climbSubsystem;

    public DrivebaseSubsystem drivebaseSubsystem;

    public DriverVisionSubsystem frontVisionSubsystem;

    public ShooterVisionSubsystem goalVisionSubsystem;

    public IndexSubsystem indexSubsystem;

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
                    hardware.backLeftModule, hardware.backRightModule, hardware.navX);
        if (DRIVER_VIS_ENABLED)
            frontVisionSubsystem = new DriverVisionSubsystem(hardware.frontCamera);
        if (GOAL_VIS_ENABLED)
            goalVisionSubsystem = new ShooterVisionSubsystem();
        if (INDEX_ENABLED)
            indexSubsystem = new IndexSubsystem(hardware.indexMotor);
        if (INTAKE_ENABLED)
            intakeSubsystem = new IntakeSubsystem(hardware.intakeMotor1, hardware.intakeMotor2,
                    hardware.intakeSolenoid, hardware.leftIntakeColorSensor,
                    hardware.rightIntakeColorSensor, hardware.centerIntakeColorSensor);
        if (SHOOTER_ENABLED)
            shooterSubsystem = new ShooterSubsystem(hardware.flywheelMotor1, hardware.flywheelMotor2,
                    hardware.turretMotor, hardware.hoodMotor);
        if (MONITOR_ENABLED) {
            monitoringSubsystem = new MonitoringSubsystem(hardware.powerDistributionPanel);
        }
    }
}
