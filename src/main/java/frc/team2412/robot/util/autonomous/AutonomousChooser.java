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
import frc.team2412.robot.commands.autonomous.OneBallAutoCommand;
import frc.team2412.robot.commands.autonomous.TwoBallAutoCommandMiddle;
import frc.team2412.robot.commands.autonomous.TwoBallScuffedAutoCommand;
import frc.team2412.robot.commands.climb.ClimbTestCommand;
import frc.team2412.robot.commands.intake.IntakeInCommand;
import frc.team2412.robot.commands.intake.IntakeTestCommand;
import frc.team2412.robot.commands.shooter.FullShootCommand;
import frc.team2412.robot.commands.shooter.ShooterTurretSetAngleCommand;
import frc.team2412.robot.subsystem.ShooterSubsystem;

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

        autonomousTab.add("Choose Auto Mode", autonomousModeChooser).withPosition(0, 0).withSize(2, 1);
    }

    public CommandBase getCommand() {
        return autonomousModeChooser.getSelected().commandSupplier.getCommand(trajectories);
    }

    private static SequentialCommandGroup getSquarePathAutoCommand(AutonomousTrajectories trajectories) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(new Follow2910TrajectoryCommand(trajectories.getSquarePathAuto()));

        return command;
    }

    private static SequentialCommandGroup getStarPathAutoCommand(AutonomousTrajectories trajectories) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(new Follow2910TrajectoryCommand(trajectories.getStarPathAuto()));

        return command;
    }

    private static SequentialCommandGroup getAutoWPICommand() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(new AutonomousCommand().getAutonomousCommand());
        return command;
    }

    private static SequentialCommandGroup getLineAutoCommand(AutonomousTrajectories trajectories) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(new Follow2910TrajectoryCommand(trajectories.getLinePathAuto()));
        return command;
    }

    @Immutable
    @FunctionalInterface
    private interface CommandSupplier {
        CommandBase getCommand(AutonomousTrajectories trajectories);
    }

    public enum AutonomousMode {
                                // Replace with individual testing commands

                                ONE_BALL((trajectories) -> new OneBallAutoCommand(), "One ball auto",
                                        Subsystems.SubsystemConstants.INDEX_ENABLED
                                                && Subsystems.SubsystemConstants.SHOOTER_ENABLED
                                                && Subsystems.SubsystemConstants.DRIVE_ENABLED),
                                TWO_BALL((trajectories) -> new TwoBallAutoCommandMiddle(), "TWo ball auto",
                                        Subsystems.SubsystemConstants.INDEX_ENABLED
                                                && Subsystems.SubsystemConstants.SHOOTER_ENABLED
                                                && Subsystems.SubsystemConstants.SHOOTER_VISION_ENABLED
                                                && Subsystems.SubsystemConstants.DRIVE_ENABLED
                                                && Subsystems.SubsystemConstants.INTAKE_ENABLED),
                                TWO_SCUFFED((trajectories) -> new TwoBallScuffedAutoCommand(), "TWO SCUFFED",
                                        true),

                                SQUARE_PATH(
                                        (trajectories) -> AutonomousChooser
                                                .getSquarePathAutoCommand(trajectories),
                                        "Square Path", Subsystems.SubsystemConstants.DRIVE_ENABLED),
                                LINE_PATH(
                                        (trajectories) -> AutonomousChooser.getLineAutoCommand(trajectories),
                                        "Line Path", Subsystems.SubsystemConstants.DRIVE_ENABLED),
                                STAR_PATH(
                                        (trajectories) -> AutonomousChooser
                                                .getStarPathAutoCommand(trajectories),
                                        "Star Path", Subsystems.SubsystemConstants.DRIVE_ENABLED),
                                WPI_PATH((trajectories) -> AutonomousChooser.getAutoWPICommand(),
                                        "WPI Lib Path", Subsystems.SubsystemConstants.DRIVE_ENABLED),
                                CLIMB((trajectories) -> new ClimbTestCommand(), "Climb test",
                                        Subsystems.SubsystemConstants.CLIMB_ENABLED),
                                INDEX((trajectories) -> new IntakeInCommand(), "Index test",
                                        Subsystems.SubsystemConstants.INDEX_ENABLED),
                                INTAKE((trajectories) -> new IntakeTestCommand(), "Intake test",
                                        Subsystems.SubsystemConstants.INTAKE_ENABLED
                                                && Subsystems.SubsystemConstants.INDEX_ENABLED),
                                SHOOTER((trajectories) -> new ShooterTurretSetAngleCommand(
                                        ShooterSubsystem.instance.getTurretTestAngle()), "Shooter test",
                                        Subsystems.SubsystemConstants.SHOOTER_ENABLED),
                                INTAKE_SHOOTER((trajectories) -> new FullShootCommand(), "Intake and shoot",
                                        Subsystems.SubsystemConstants.INTAKE_ENABLED
                                                && Subsystems.SubsystemConstants.INDEX_ENABLED
                                                && Subsystems.SubsystemConstants.SHOOTER_ENABLED);

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
