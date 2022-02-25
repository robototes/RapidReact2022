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

        autonomousModeChooser.setDefaultOption(AutonomousMode.SQUARE_PATH.uiName, AutonomousMode.SQUARE_PATH);

        for (var mode : AutonomousMode.values()) {
            if (mode != AutonomousMode.SQUARE_PATH) {
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

    @Immutable
    @FunctionalInterface
    private interface CommandSupplier {
        CommandBase getCommand(Subsystems subsystems, AutonomousTrajectories trajectories);
    }

    public enum AutonomousMode {
        // Replace with individual testing commands
        // spotless:off
        SQUARE_PATH((subsystems, trajectories) -> AutonomousChooser.getSquarePathAutoCommand(subsystems, trajectories), "Square Path"),
        STAR_PATH((subsystems, trajectories) -> AutonomousChooser.getStarPathAutoCommand(subsystems, trajectories), "Star Path"),
        WPI_PATH((subsystems, trajectories) -> AutonomousChooser.getAutoWPICommand(subsystems), "WPI Lib Path"),
        CLIMB((subsystems, trajectories) -> new ClimbTestCommand(subsystems.climbSubsystem), "Climb test"),
        INDEX((subsystems, trajectories) -> new IntakeInCommand(subsystems.indexSubsystem, subsystems.intakeSubsystem), "Index test"),
        INTAKE((subsystems, trajectories) -> new IntakeTestCommand(subsystems.intakeSubsystem), "Intake test"),
        SHOOTER((subsystems, trajectories) -> new ShooterTargetCommand(subsystems.shooterSubsystem, subsystems.shooterVisionSubsystem), "Shooter test"),
        INTAKE_SHOOTER((subsystems, trajectories) -> new FullShootCommand(subsystems.shooterSubsystem, subsystems.shooterVisionSubsystem, subsystems.intakeSubsystem, subsystems.indexSubsystem), "Intake and shoot");
        // spotless:on

        public final CommandSupplier commandSupplier;
        public final String uiName;

        private AutonomousMode(CommandSupplier commandSupplier, String uiName) {
            this.commandSupplier = commandSupplier;
            this.uiName = uiName;
        }
    }
}
