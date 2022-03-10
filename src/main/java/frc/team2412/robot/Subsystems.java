package frc.team2412.robot;

import static frc.team2412.robot.Subsystems.SubsystemConstants.CLIMB_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.DRIVE_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.INDEX_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.INTAKE_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.SHOOTER_ENABLED;

import frc.team2412.robot.subsystem.ClimbSubsystem;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;
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
    }

    public final Hardware hardware;

    public ClimbSubsystem climbSubsystem;

    public DrivebaseSubsystem drivebaseSubsystem;

    public ShooterVisionSubsystem shooterVisionSubsystem;

    public IndexSubsystem indexSubsystem;

    public IntakeSubsystem intakeSubsystem;

    public ShooterSubsystem shooterSubsystem;

    private Subsystems() {
        boolean comp = Robot.instance.isCompetition();

        hardware = Hardware.instance;

        if (DRIVE_ENABLED)
            drivebaseSubsystem = DrivebaseSubsystem.instance;
        if (!comp)
            return;
        if (CLIMB_ENABLED)
            climbSubsystem = ClimbSubsystem.instance;
        if (INTAKE_ENABLED)
            intakeSubsystem = IntakeSubsystem.instance;
        if (SHOOTER_ENABLED) {
            shooterSubsystem = ShooterSubsystem.instance;
            shooterVisionSubsystem = ShooterVisionSubsystem.instance;
        }

        if (INDEX_ENABLED) {
            indexSubsystem = IndexSubsystem.instance;
        }
    }

    // Singleton
    public static final Subsystems instance = new Subsystems();
}
