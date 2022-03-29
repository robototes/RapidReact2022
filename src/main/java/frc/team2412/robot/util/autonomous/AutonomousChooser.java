package frc.team2412.robot.util.autonomous;

import com.google.errorprone.annotations.Immutable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.Subsystems;
import frc.team2412.robot.commands.autonomous.*;
import frc.team2412.robot.commands.autonomous.debug.LinePath;
import frc.team2412.robot.commands.autonomous.debug.SquarePath;
import frc.team2412.robot.commands.autonomous.debug.StarPath;
import frc.team2412.robot.commands.climb.ClimbExtendSlowlyCommand;
import frc.team2412.robot.commands.climb.ClimbRetractSlowlyCommand;
import frc.team2412.robot.commands.climb.ClimbTestCommand;
import frc.team2412.robot.commands.diagnostic.DiagnosticIntakeCommandGroup;
import frc.team2412.robot.commands.index.IndexTestCommand;
import frc.team2412.robot.commands.shooter.FullShootCommand;
import frc.team2412.robot.commands.shooter.ShooterTurretSetAngleCommand;

public class AutonomousChooser {

    private final SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();
    public final Subsystems subsystems;

    public AutonomousChooser(Subsystems subsystems) {
        this.subsystems = subsystems;

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
        AutonomousMode autoMode = autonomousModeChooser.getSelected();
        return autoMode != null
                ? autoMode.commandSupplier.getCommand(subsystems)
                : null;
    }

    public Pose2d getStartPose() {
        return autonomousModeChooser.getSelected().startPose;
    }

    @Immutable
    @FunctionalInterface
    private interface CommandSupplier {
        CommandBase getCommand(Subsystems subsystems);
    }

    public enum AutonomousMode {
        // Replace with individual testing commands
        ONE_BALL(
                (subsystems) -> new OneBallAutoCommand(subsystems.indexSubsystem,
                        subsystems.shooterSubsystem, subsystems.targetLocalizer, subsystems.drivebaseSubsystem,
                        subsystems.intakeSubsystem),
                "One ball auto",
                Subsystems.SubsystemConstants.INDEX_ENABLED &&
                        Subsystems.SubsystemConstants.SHOOTER_ENABLED &&
                        Subsystems.SubsystemConstants.DRIVE_ENABLED &&
                        Subsystems.SubsystemConstants.INTAKE_ENABLED),
        TWO_BALL(
                (subsystems) -> new TwoBallAutoCommandLeft(subsystems.indexSubsystem,
                        subsystems.shooterSubsystem,
                        subsystems.targetLocalizer, subsystems.drivebaseSubsystem,
                        subsystems.intakeSubsystem),
                "Two ball auto",
                Subsystems.SubsystemConstants.INDEX_ENABLED &&
                        Subsystems.SubsystemConstants.INTAKE_ENABLED &&
                        Subsystems.SubsystemConstants.SHOOTER_ENABLED &&
                        Subsystems.SubsystemConstants.SHOOTER_VISION_ENABLED &&
                        Subsystems.SubsystemConstants.DRIVE_ENABLED &&
                        Subsystems.SubsystemConstants.INTAKE_ENABLED,
                new Pose2d(new Translation2d(359, 209), new Rotation2d(Math.PI))),
        SQUARE_PATH((subsystems) -> new SquarePath(subsystems.drivebaseSubsystem),
                "Square Path", Subsystems.SubsystemConstants.DRIVE_ENABLED),
        LINE_PATH((subsystems) -> new LinePath(subsystems.drivebaseSubsystem),
                "Line Path", Subsystems.SubsystemConstants.DRIVE_ENABLED),
        STAR_PATH((subsystems) -> new StarPath(subsystems.drivebaseSubsystem),
                "Star Path", Subsystems.SubsystemConstants.DRIVE_ENABLED),
        TWO_BALL_FENDER((subsystems) -> new TwoBallFenderAutoCommand(subsystems.drivebaseSubsystem),
                "Two ball fender path", Subsystems.SubsystemConstants.DRIVE_ENABLED,
                new Pose2d(new Translation2d(231.8, 200.8), Rotation2d.fromDegrees(46))),
        JACK_FIVE_BALL((subsystems) -> new JackFiveBallAutoCommand(subsystems.drivebaseSubsystem, subsystems.indexSubsystem, subsystems.shooterSubsystem, subsystems.targetLocalizer),
                "2910 Five ball path", Subsystems.SubsystemConstants.DRIVE_ENABLED,
                new Pose2d(new Translation2d(328, 75.551), Rotation2d.fromDegrees(-90))),
        WPI_PATH((subsystems) -> new WPILibFiveBallAutoCommand(subsystems.drivebaseSubsystem), 
                "WPI Lib Path", Subsystems.SubsystemConstants.DRIVE_ENABLED,
                new Pose2d(new Translation2d(331, 71), new Rotation2d(Math.PI))),
        CLIMB((subsystems) -> new ClimbTestCommand(subsystems.climbSubsystem), 
                "Climb test", Subsystems.SubsystemConstants.CLIMB_ENABLED),
        INDEX((subsystems) -> new IndexTestCommand(subsystems.indexSubsystem),
                "Index test", Subsystems.SubsystemConstants.INDEX_ENABLED),
        INTAKE((subsystems) -> new DiagnosticIntakeCommandGroup(subsystems.intakeSubsystem),
                "Intake test", Subsystems.SubsystemConstants.INTAKE_ENABLED),
        SHOOTER((subsystems) -> new ShooterTurretSetAngleCommand(subsystems.shooterSubsystem,
                subsystems.shooterSubsystem.getTurretTestAngle()), "Shooter test",
                Subsystems.SubsystemConstants.SHOOTER_ENABLED),
        INTAKE_SHOOTER(
                (subsystems) -> new FullShootCommand(subsystems.shooterSubsystem,
                        subsystems.targetLocalizer, subsystems.intakeSubsystem, subsystems.indexSubsystem),
                "Intake and shoot",
                Subsystems.SubsystemConstants.INTAKE_ENABLED &&
                        Subsystems.SubsystemConstants.INDEX_ENABLED &&
                        Subsystems.SubsystemConstants.SHOOTER_ENABLED),
        CLIMB_DOWN_IN_QUEUE(
                (subsystems) -> new ClimbRetractSlowlyCommand(subsystems.climbSubsystem,
                        subsystems.intakeSubsystem, subsystems.indexSubsystem, subsystems.shooterSubsystem,
                        subsystems.drivebaseSubsystem),
                "Climb down in queue",
                Subsystems.SubsystemConstants.CLIMB_ENABLED &&
                        Subsystems.SubsystemConstants.INTAKE_ENABLED &&
                        Subsystems.SubsystemConstants.INDEX_ENABLED &&
                        Subsystems.SubsystemConstants.SHOOTER_ENABLED &&
                        Subsystems.SubsystemConstants.DRIVE_ENABLED),
        CLIMB_UP_IN_QUEUE(
                (subsystems) -> new ClimbExtendSlowlyCommand(subsystems.climbSubsystem,
                        subsystems.intakeSubsystem, subsystems.indexSubsystem, subsystems.shooterSubsystem),
                "Climb up in queue",
                Subsystems.SubsystemConstants.CLIMB_ENABLED &&
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
