package frc.team2412.robot.util.autonomous;

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
import frc.team2412.robot.commands.shooter.ShooterTurretSetAngleCommand;
import frc.team2412.robot.commands.autonomous.OneBallAutoCommand;
import frc.team2412.robot.commands.autonomous.TwoBallAutoCommandMiddle;

import static frc.team2412.robot.Subsystems.SubsystemConstants.*;

public class AutonomousChooser {

    private final SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();
    public final Subsystems subsystems;
    private final AutonomousTrajectories trajectories;

    public AutonomousChooser(Subsystems subsystems, AutonomousTrajectories trajectories) {
        this.subsystems = subsystems;
        this.trajectories = trajectories;

        boolean setDefault = false;
        for (var mode : AutonomousMode.values()) {
            if (mode != AutonomousMode.SQUARE_PATH) {
                if (mode.enabled) {
                    if (!setDefault) {
                        autonomousModeChooser.setDefaultOption(mode.uiName, mode);
                        setDefault = true;
                    } else
                        autonomousModeChooser.addOption(mode.uiName, mode);
                }
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


        ONE_BALL((subsystems, trajectories) -> new OneBallAutoCommand(subsystems.indexSubsystem, subsystems.shooterSubsystem, subsystems.targetLocalizer, subsystems.drivebaseSubsystem),
            "One ball auto",
            INDEX_ENABLED && SHOOTER_ENABLED && DRIVE_ENABLED),
            TWO_BALL((subsystems, trajectories) -> new TwoBallAutoCommandMiddle(subsystems.indexSubsystem, subsystems.shooterSubsystem, subsystems.targetLocalizer, subsystems.drivebaseSubsystem, subsystems.intakeSubsystem),
            "TWo ball auto",
            INDEX_ENABLED && SHOOTER_ENABLED && DRIVE_ENABLED && INTAKE_ENABLED),

        //these can be method refs because only one will be used at once
        SQUARE_PATH(AutonomousChooser::getSquarePathAutoCommand, "Square Path", DRIVE_ENABLED),
        LINE_PATH(AutonomousChooser::getLineAutoCommand, "Line Path", DRIVE_ENABLED),
        STAR_PATH(AutonomousChooser::getStarPathAutoCommand, "Star Path", DRIVE_ENABLED),
        WPI_PATH((subsystems, trajectories) -> AutonomousChooser.getAutoWPICommand(subsystems), "WPI Lib Path", DRIVE_ENABLED),
        CLIMB((subsystems, trajectories) -> new ClimbTestCommand(subsystems.climbSubsystem), "Climb test", CLIMB_ENABLED),
        INDEX((subsystems, trajectories) -> new IntakeInCommand(subsystems.indexSubsystem, subsystems.intakeSubsystem), "Index test", INDEX_ENABLED),
        INTAKE((subsystems, trajectories) -> new IntakeTestCommand(subsystems.intakeSubsystem), "Intake test", INTAKE_ENABLED && INDEX_ENABLED),
        SHOOTER((subsystems, trajectories) -> new ShooterTurretSetAngleCommand(subsystems.shooterSubsystem, subsystems.shooterSubsystem.getTurretTestAngle()), "Shooter test", SHOOTER_ENABLED),
        INTAKE_SHOOTER((subsystems, trajectories) -> new FullShootCommand(subsystems.shooterSubsystem, subsystems.targetLocalizer, subsystems.intakeSubsystem, subsystems.indexSubsystem),
            "Intake and shoot",
            INTAKE_ENABLED && INDEX_ENABLED && SHOOTER_ENABLED);
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
