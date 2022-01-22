package frc.team2412.robot;

import static frc.team2412.robot.SubsystemContainer.SubsystemConstants.*;

import frc.team2412.robot.Subsystems.*;

public class SubsystemContainer {
    public static class SubsystemConstants {
        public static final boolean CLIMB_ENABLED = false;
        public static final boolean DRIVE_ENABLED = false;
        public static final boolean FRONT_VIS_ENABLED = false;
        public static final boolean GOAL_VIS_ENABLED = false;
        public static final boolean INDEX_ENABLED = false;
        public static final boolean INTAKE_ENABLED = false; 
        public static final boolean SHOOTER_ENABLED = false;


    }
    public final Hardware hardware;

    public ClimbSubsystem climbSubsystem;

    public DrivebaseSubsystem drivebaseSubsystem;

    public FrontVisionSubsystem frontVisionSubsystem;

    public GoalVisionSubsystem goalVisionSubsystem;

    public IndexSubsystem indexSubsystem;

    public IntakeSubsystem intakeSubsystem;

    public ShooterSubsystem shooterSubsystem;

    public SubsystemContainer(Hardware h){
        hardware = h;
        if(CLIMB_ENABLED) climbSubsystem = new ClimbSubsystem();
        if(DRIVE_ENABLED) drivebaseSubsystem = new DrivebaseSubsystem();
        if(FRONT_VIS_ENABLED) frontVisionSubsystem = new FrontVisionSubsystem();
        if(GOAL_VIS_ENABLED) goalVisionSubsystem = new GoalVisionSubsystem();
        if(INDEX_ENABLED) indexSubsystem = new IndexSubsystem();
        if(INTAKE_ENABLED) intakeSubsystem = new IntakeSubsystem();
        if(SHOOTER_ENABLED) shooterSubsystem = new ShooterSubsystem();
    }
}
