package frc.team2412.robot;

import static frc.team2412.robot.Subsystems.SubsystemConstants.*;

import frc.team2412.robot.subsystem.ClimbSubsystem;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.ShooterVisionSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Subsystems implements Loggable {
    public static class SubsystemConstants {
        public static final boolean CLIMB_ENABLED = true;
        public static final boolean DRIVE_ENABLED = true;
        public static final boolean DRIVER_VIS_ENABLED = false;
        public static final boolean SHOOTER_VISION_ENABLED = false;
        public static final boolean INDEX_ENABLED = false;
        public static final boolean INTAKE_ENABLED = false;
        public static final boolean SHOOTER_ENABLED = false;
        public static final boolean SHOOTER_TESTING = false;
    }

    public final Hardware hardware;

    public ClimbSubsystem climbSubsystem;

    public DrivebaseSubsystem drivebaseSubsystem;

    public ShooterVisionSubsystem shooterVisionSubsystem;

    public IndexSubsystem indexSubsystem;

    @Log(tabName = "IntakeSubsystem")
    public IntakeSubsystem intakeSubsystem;

    public ShooterSubsystem shooterSubsystem;

    private Subsystems(Hardware hardware) {
        boolean comp = Robot.instance.isCompetition();

        this.hardware = hardware;

        if (DRIVE_ENABLED)
            drivebaseSubsystem = DrivebaseSubsystem.instance;
        if (!comp)
            return;
        if (CLIMB_ENABLED)
            climbSubsystem = ClimbSubsystem.instance;
        if (SHOOTER_VISION_ENABLED)
            shooterVisionSubsystem = ShooterVisionSubsystem.instance;
        if (INTAKE_ENABLED)
            intakeSubsystem = IntakeSubsystem.instance;
        if (SHOOTER_ENABLED)
            shooterSubsystem = ShooterSubsystem.instance;
        if (INDEX_ENABLED) {
            indexSubsystem = IndexSubsystem.instance;
        }
    }

    // Singleton
    public static final Subsystems instance = new Subsystems(Hardware.instance);
}
