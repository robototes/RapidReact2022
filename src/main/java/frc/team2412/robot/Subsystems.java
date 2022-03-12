package frc.team2412.robot;

import static frc.team2412.robot.Subsystems.SubsystemConstants.*;

import frc.team2412.robot.subsystem.ClimbSubsystem;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;
import frc.team2412.robot.subsystem.PneumaticHubSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.ShooterVisionSubsystem;
import io.github.oblarg.oblog.Loggable;

public class Subsystems implements Loggable {
    public static class SubsystemConstants {
        public static final boolean CLIMB_ENABLED = true;
        public static final boolean DRIVE_ENABLED = true;
        public static final boolean DRIVER_VIS_ENABLED = false;
        public static final boolean SHOOTER_VISION_ENABLED = true;
        public static final boolean INDEX_ENABLED = true;
        public static final boolean INTAKE_ENABLED = true;
        public static final boolean SHOOTER_ENABLED = true;
        public static final boolean SHOOTER_TESTING = false;
        public static final boolean PNEUMATICS_ENABLED = true;
    }

    public final Hardware hardware;

    public ClimbSubsystem climbSubsystem;

    public DrivebaseSubsystem drivebaseSubsystem;

    public ShooterVisionSubsystem shooterVisionSubsystem;

    public IndexSubsystem indexSubsystem;

    public IntakeSubsystem intakeSubsystem;

    public ShooterSubsystem shooterSubsystem;

    public PneumaticHubSubsystem pneumaticHubSubsystem;

    public Subsystems(Hardware h) {
        boolean comp = Robot.getInstance().isCompetition();

        hardware = h;

        if (DRIVE_ENABLED)
            drivebaseSubsystem = new DrivebaseSubsystem(hardware.frontLeftModule, hardware.frontRightModule,
                    hardware.backLeftModule, hardware.backRightModule, hardware.gyro,
                    Hardware.HardwareConstants.MODULE_MAX_VELOCITY_METERS_PER_SEC);
        if (!comp)
            return;

        if (PNEUMATICS_ENABLED) {
            pneumaticHubSubsystem = new PneumaticHubSubsystem();
        }
        if (CLIMB_ENABLED)
            climbSubsystem = new ClimbSubsystem(hardware.climbMotorFixed, hardware.climbMotorDynamic,
                    hardware.climbAngle);
        if (INTAKE_ENABLED)
            intakeSubsystem = new IntakeSubsystem(hardware.intakeMotor, hardware.intakeMotor2, hardware.intakeSolenoid);
        if (SHOOTER_ENABLED) {
            shooterSubsystem = new ShooterSubsystem(hardware.flywheelMotor1, hardware.flywheelMotor2,
                    hardware.turretMotor, hardware.hoodMotor);
            shooterVisionSubsystem = new ShooterVisionSubsystem();
        }

        if (INDEX_ENABLED) {
            indexSubsystem = new IndexSubsystem(hardware.ingestIndexMotor, hardware.feederIndexMotor,
                    hardware.ingestProximity, hardware.feederProximity, hardware.ingestBlueColor,
                    hardware.ingestRedColor, hardware.feederBlueColor, hardware.feederRedColor,
                    hardware.ingestTopProximity,
                    hardware.ingestTopBlueColor,
                    hardware.ingestTopRedColor);
        }
    }
}
