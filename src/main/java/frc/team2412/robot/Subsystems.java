package frc.team2412.robot;

import static frc.team2412.robot.Subsystems.SubsystemConstants.CLIMB_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.DRIVE_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.INDEX_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.INTAKE_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.SHOOTER_ENABLED;

import frc.team2412.robot.subsystem.*;
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
    }

    public ClimbSubsystem climbSubsystem;

    public DrivebaseSubsystem drivebaseSubsystem;

    public ShooterVisionSubsystem shooterVisionSubsystem;

    public IndexSubsystem indexSubsystem;

    public IntakeSubsystem intakeSubsystem;

    public ShooterSubsystem shooterSubsystem;

    public TargetLocalizer targetLocalizer;

    public Subsystems() {
        boolean comp = Robot.getInstance().isCompetition();

        if (DRIVE_ENABLED) {
            drivebaseSubsystem = new DrivebaseSubsystem();
        }
        if (!comp) {
            return;
        }
        if (CLIMB_ENABLED) {
            climbSubsystem = new ClimbSubsystem();
        }
        if (INDEX_ENABLED) {
            indexSubsystem = new IndexSubsystem();
        }
        if (INTAKE_ENABLED) {
            intakeSubsystem = new IntakeSubsystem();
        }
        if (SHOOTER_ENABLED) {
            shooterSubsystem = new ShooterSubsystem();
            shooterVisionSubsystem = new ShooterVisionSubsystem();
            targetLocalizer = new TargetLocalizer(drivebaseSubsystem, shooterSubsystem, shooterVisionSubsystem);
        }
    }
}
