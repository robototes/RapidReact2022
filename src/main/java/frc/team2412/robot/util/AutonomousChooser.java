package frc.team2412.robot.util;

import com.google.errorprone.annotations.Immutable;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.Subsystems;
import frc.team2412.robot.commands.autonomous.AutonomousCommand;
import frc.team2412.robot.commands.autonomous.Follow2910TrajectoryCommand;
import frc.team2412.robot.commands.climb.ClimbTestCommand;
import frc.team2412.robot.commands.intake.IntakeInCommand;
import frc.team2412.robot.commands.intake.IntakeTestCommand;
import frc.team2412.robot.commands.shooter.FullShootCommand;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;

public class AutonomousChooser {

    private final SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();
    public final Subsystems subsystems;
    private final AutonomousTrajectories trajectories;

    public AutonomousChooser(Subsystems subsystems, AutonomousTrajectories trajectories) {
        this.subsystems = subsystems;
        this.trajectories = trajectories;

        if (AutonomousMode.SQUARE_PATH.enabled)
            autonomousModeChooser.setDefaultOption(AutonomousMode.SQUARE_PATH.uiName, AutonomousMode.SQUARE_PATH);

        for (var mode : AutonomousMode.values()) {
            if (mode != AutonomousMode.SQUARE_PATH) {
                if (mode.enabled)
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

    private static SequentialCommandGroup getAutoWPICommand(Subsystems subsystems) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(new AutonomousCommand(subsystems.drivebaseSubsystem).getAutonomousCommand());
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
        // spotless:off
        SQUARE_PATH((subsystems, trajectories) -> AutonomousChooser.getSquarePathAutoCommand(subsystems, trajectories), "Square Path", Subsystems.SubsystemConstants.DRIVE_ENABLED),
        LINE_PATH((subsystems, trajectories) -> AutonomousChooser.getLineAutoCommand(subsystems, trajectories), "Line Path", Subsystems.SubsystemConstants.DRIVE_ENABLED),
        STAR_PATH((subsystems, trajectories) -> AutonomousChooser.getStarPathAutoCommand(subsystems, trajectories), "Star Path", Subsystems.SubsystemConstants.DRIVE_ENABLED),
        WPI_PATH((subsystems, trajectories) -> AutonomousChooser.getAutoWPICommand(subsystems), "WPI Lib Path", Subsystems.SubsystemConstants.DRIVE_ENABLED),
        CLIMB((subsystems, trajectories) -> new ClimbTestCommand(subsystems.climbSubsystem), "Climb test", Subsystems.SubsystemConstants.CLIMB_ENABLED),
        INDEX((subsystems, trajectories) -> new IntakeInCommand(subsystems.indexSubsystem, subsystems.intakeSubsystem), "Index test", Subsystems.SubsystemConstants.INDEX_ENABLED),
        INTAKE((subsystems, trajectories) -> new IntakeTestCommand(subsystems.intakeSubsystem), "Intake test", Subsystems.SubsystemConstants.INTAKE_ENABLED && Subsystems.SubsystemConstants.INDEX_ENABLED),
        SHOOTER((subsystems, trajectories) -> new ShooterTargetCommand(subsystems.shooterSubsystem, subsystems.shooterVisionSubsystem), "Shooter test", Subsystems.SubsystemConstants.SHOOTER_ENABLED),
        INTAKE_SHOOTER((subsystems, trajectories) -> new FullShootCommand(subsystems.shooterSubsystem, subsystems.shooterVisionSubsystem, subsystems.intakeSubsystem, subsystems.indexSubsystem), 
            "Intake and shoot", 
            Subsystems.SubsystemConstants.INTAKE_ENABLED && 
            Subsystems.SubsystemConstants.INDEX_ENABLED &&
            Subsystems.SubsystemConstants.SHOOTER_ENABLED &&
            Subsystems.SubsystemConstants.SHOOTER_VISION_ENABLED);
        // spotless:on

        public final CommandSupplier commandSupplier;
        public final String uiName;
        public final boolean enabled;

        private AutonomousMode(CommandSupplier commandSupplier, String uiName, boolean enabled) {
            this.commandSupplier = commandSupplier;
            this.uiName = uiName;
            this.enabled = enabled;
        }
    }
}
