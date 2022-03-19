package frc.team2412.robot.util.autonomous;

import com.google.errorprone.annotations.Immutable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.Subsystems;
import frc.team2412.robot.commands.autonomous.*;
import frc.team2412.robot.commands.climb.ClimbTestCommand;
import frc.team2412.robot.commands.intake.IntakeIndexInCommand;
import frc.team2412.robot.commands.intake.IntakeTestCommand;
import frc.team2412.robot.commands.shooter.FullShootCommand;
import frc.team2412.robot.commands.shooter.ShooterTurretSetAngleCommand;

import java.util.List;

public class AutonomousChooser {

    private final SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();
    public final Subsystems subsystems;
    private final AutonomousTrajectories trajectories;

    public AutonomousChooser(Subsystems subsystems, AutonomousTrajectories trajectories) {
        this.subsystems = subsystems;
        this.trajectories = trajectories;

        boolean setDefault = false;
        for (var mode : AutonomousMode.values()) {
            if (mode.enabled) {
                if (!setDefault) {
                    autonomousModeChooser.setDefaultOption(mode.uiName, mode);
                    setDefault = true;
                } else
                    autonomousModeChooser.addOption(mode.uiName, mode);
            }
        }

        ShuffleboardTab autonomousTab = Shuffleboard.getTab("Autonomous");

        autonomousTab.add("Choose Auto Mode", autonomousModeChooser)
                .withPosition(0, 0)
                .withSize(2, 1);
    }

    public CommandBase getCommand() {
        return autonomousModeChooser.getSelected().commandSupplier.getCommand(subsystems, trajectories);
    }

    public Pose2d getStartPose() {
        return autonomousModeChooser.getSelected().startPose;
    }

    private static SequentialCommandGroup getSquarePathAutoCommand(Subsystems subsystems,
            AutonomousTrajectories trajectories) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(
                new Follow2910TrajectoryCommand(subsystems.drivebaseSubsystem, trajectories.getSquarePathAuto()));

        return command;
    }

    private static SequentialCommandGroup getStarPathAutoCommand(Subsystems subsystems,
            AutonomousTrajectories trajectories) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(
                new Follow2910TrajectoryCommand(subsystems.drivebaseSubsystem, trajectories.getStarPathAuto()));

        return command;
    }


    private static SequentialCommandGroup getLineAutoCommand(Subsystems subsystems,
            AutonomousTrajectories trajectories) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(
                new Follow2910TrajectoryCommand(subsystems.drivebaseSubsystem, trajectories.getLinePathAuto()));
        return command;
    }

    @Immutable
    @FunctionalInterface
    private interface CommandSupplier {
        CommandBase getCommand(Subsystems subsystems, AutonomousTrajectories trajectories);
    }

    public enum AutonomousMode {
        // Replace with individual testing commands
        ONE_BALL(
                (subsystems, trajectories) -> new OneBallAutoCommand(subsystems.indexSubsystem,
                        subsystems.shooterSubsystem, subsystems.targetLocalizer, subsystems.drivebaseSubsystem),
                "One ball auto",
                Subsystems.SubsystemConstants.INDEX_ENABLED &&
                        Subsystems.SubsystemConstants.SHOOTER_ENABLED &&
                        Subsystems.SubsystemConstants.DRIVE_ENABLED),
        TWO_BALL(
                (subsystems, trajectories) -> new TwoBallAutoCommandLeft(subsystems.indexSubsystem,
                        subsystems.shooterSubsystem,
                        subsystems.targetLocalizer, subsystems.drivebaseSubsystem,
                        subsystems.intakeSubsystem),
                "Two ball auto",
                Subsystems.SubsystemConstants.INDEX_ENABLED &&
                        Subsystems.SubsystemConstants.INTAKE_ENABLED &&
                        Subsystems.SubsystemConstants.SHOOTER_ENABLED &&
                        Subsystems.SubsystemConstants.DRIVE_ENABLED),
        SQUARE_PATH((subsystems, trajectories) -> AutonomousChooser.getSquarePathAutoCommand(subsystems, trajectories),
                "Square Path", Subsystems.SubsystemConstants.DRIVE_ENABLED),
        LINE_PATH((subsystems, trajectories) -> AutonomousChooser.getLineAutoCommand(subsystems, trajectories),
                "Line Path", Subsystems.SubsystemConstants.DRIVE_ENABLED),
        STAR_PATH((subsystems, trajectories) -> AutonomousChooser.getStarPathAutoCommand(subsystems, trajectories),
                "Star Path", Subsystems.SubsystemConstants.DRIVE_ENABLED),
        TWO_BALL_FENDER((subsystems, trajectories) -> new TwoBallFenderAutoCommand(subsystems.drivebaseSubsystem),
                "Two ball fender path", Subsystems.SubsystemConstants.DRIVE_ENABLED,
                new Pose2d(new Translation2d(231.8, 200.8), Rotation2d.fromDegrees(46))),
        FIVE_BALL((subsystems, trajectories) -> new FiveBallAutoCommand(subsystems.drivebaseSubsystem),
                "Five ball path", Subsystems.SubsystemConstants.DRIVE_ENABLED,
                new Pose2d(new Translation2d(295.2, 74), Rotation2d.fromDegrees(0))),
        CLIMB((subsystems, trajectories) -> new ClimbTestCommand(subsystems.climbSubsystem), "Climb test",
                Subsystems.SubsystemConstants.CLIMB_ENABLED),
        INDEX((subsystems, trajectories) -> new IntakeIndexInCommand(subsystems.indexSubsystem,
                subsystems.intakeSubsystem),
                "Index test", Subsystems.SubsystemConstants.INDEX_ENABLED),
        INTAKE((subsystems, trajectories) -> new IntakeTestCommand(subsystems.intakeSubsystem), "Intake test",
                Subsystems.SubsystemConstants.INTAKE_ENABLED && Subsystems.SubsystemConstants.INDEX_ENABLED),
        SHOOTER((subsystems, trajectories) -> new ShooterTurretSetAngleCommand(subsystems.shooterSubsystem,
                subsystems.shooterSubsystem.getTurretTestAngle()), "Shooter test",
                Subsystems.SubsystemConstants.SHOOTER_ENABLED),
        INTAKE_SHOOTER(
                (subsystems, trajectories) -> new FullShootCommand(subsystems.shooterSubsystem,
                        subsystems.targetLocalizer, subsystems.intakeSubsystem, subsystems.indexSubsystem),
                "Intake and shoot",
                Subsystems.SubsystemConstants.INTAKE_ENABLED &&
                        Subsystems.SubsystemConstants.INDEX_ENABLED &&
                        Subsystems.SubsystemConstants.SHOOTER_ENABLED);

        public final CommandSupplier commandSupplier;
        public final String uiName;
        public final boolean enabled;
        public final Pose2d startPose;

        private AutonomousMode(CommandSupplier commandSupplier, String uiName, boolean enabled, Pose2d startPose) {
            this.commandSupplier = commandSupplier;
            this.uiName = uiName;
            this.enabled = enabled;
            this.startPose = startPose;
        }

        private AutonomousMode(CommandSupplier commandSupplier, String uiName, boolean enabled) {
            this.commandSupplier = commandSupplier;
            this.uiName = uiName;
            this.enabled = enabled;
            this.startPose = new Pose2d();
        }
    }
}
