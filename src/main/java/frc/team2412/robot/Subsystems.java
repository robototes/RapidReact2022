package frc.team2412.robot;

import frc.team2412.robot.subsystem.*;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.team2412.robot.Subsystems.SubsystemConstants.*;

public class Subsystems implements Loggable {
    public static class SubsystemConstants {
        public static final boolean CLIMB_ENABLED = false;
        public static final boolean DRIVE_ENABLED = false;
        public static final boolean DRIVER_VIS_ENABLED = false;
        public static final boolean SHOOTER_VISION_ENABLED = true;
        public static final boolean INDEX_ENABLED = true;
        public static final boolean INTAKE_ENABLED = true;
        public static final boolean SHOOTER_ENABLED = true;
        public static final boolean I2C_MUX_ENABLED = false;
        public static final boolean SHOOTER_TESTING = true;
    }

    public final Hardware hardware;

    public ClimbSubsystem climbSubsystem;

    public DrivebaseSubsystem drivebaseSubsystem;

    public ShooterVisionSubsystem shooterVisionSubsystem;

    public IndexSubsystem indexSubsystem;

    @Log(tabName = "IntakeSubsystem")
    public IntakeSubsystem intakeSubsystem;

    public ShooterSubsystem shooterSubsystem;

    public Subsystems(Hardware h) {
        hardware = h;

        if (CLIMB_ENABLED)
            climbSubsystem = new ClimbSubsystem(hardware.climbMotorFixed, hardware.climbMotorDynamic,
                    hardware.climbAngle);
        if (DRIVE_ENABLED)
            drivebaseSubsystem = new DrivebaseSubsystem(hardware.frontLeftModule, hardware.frontRightModule,
                    hardware.backLeftModule, hardware.backRightModule, hardware.navX,
                    Hardware.HardwareConstants.MODULE_MAX_VELOCITY_METERS_PER_SEC);
        if (SHOOTER_VISION_ENABLED)
            shooterVisionSubsystem = new ShooterVisionSubsystem();
        if (INDEX_ENABLED)
            indexSubsystem = new IndexSubsystem(hardware.ingestIndexMotor, hardware.feederIndexMotor,
                    hardware.ingestProximity, hardware.feederProximity);
        if (INTAKE_ENABLED)
            intakeSubsystem = new IntakeSubsystem(hardware.intakeMotor1, hardware.intakeMotor2,
                    hardware.intakeSolenoid, hardware.leftIntakeColorSensor, hardware.rightIntakeColorSensor,
                    hardware.centerIntakeColorSensor);
        if (SHOOTER_ENABLED)
            shooterSubsystem = new ShooterSubsystem(hardware.flywheelMotor1, hardware.flywheelMotor2,
                    hardware.turretMotor, hardware.hoodMotor);
    }
}
