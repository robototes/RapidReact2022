package frc.team2412.robot;

import frc.team2412.robot.subsystems.*;

import static frc.team2412.robot.SubsystemContainer.SubsystemConstants.*;

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

    public SubsystemContainer(Hardware h) {
        hardware = h;
        if (CLIMB_ENABLED)
            climbSubsystem = new ClimbSubsystem(hardware.climbFixed1, hardware.climbFixed2, hardware.climbAngled1, hardware.climbAngled2, hardware.climbAngle);
        if (DRIVE_ENABLED)
            drivebaseSubsystem = new DrivebaseSubsystem(hardware.frontLeftModule, hardware.frontRightModule, hardware.backLeftModule, hardware.backRightModule, hardware.navX);
        if (FRONT_VIS_ENABLED)
            frontVisionSubsystem = new FrontVisionSubsystem(hardware.frontCamera);
        if (GOAL_VIS_ENABLED)
            goalVisionSubsystem = new GoalVisionSubsystem(hardware.limelight);
        if (INDEX_ENABLED)
            indexSubsystem = new IndexSubsystem(hardware.indexMotor);
        if (INTAKE_ENABLED)
            intakeSubsystem = new IntakeSubsystem(hardware.intakeMotor1, hardware.intakeMotor2, hardware.intakeSolenoid);
        if (SHOOTER_ENABLED)
            shooterSubsystem = new ShooterSubsystem(hardware.flywheelMotor1, hardware.flywheelMotor2, hardware.turretMotor, hardware.hoodMotor);
    }
}
