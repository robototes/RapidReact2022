package frc.team2412.robot.commands.autonomous;

import frc.team2412.robot.subsystem.*;
import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.commands.index.IndexShootCommand;
import frc.team2412.robot.commands.intake.IntakeSetExtendCommand;
import frc.team2412.robot.commands.intake.IntakeCommand;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;

public class TwoBallAutoCommandLeft extends SequentialCommandGroup {
    public TwoBallAutoCommandLeft(IndexSubsystem indexSubsystem, ShooterSubsystem shooterSubsystem,
            TargetLocalizer localizer, DrivebaseSubsystem drivebaseSubsystem,
            IntakeSubsystem intakeSubsystem) {
        Trajectory robotPath = new Trajectory(
                new SimplePathBuilder(new Vector2(397.308, 122.461), Rotation2.fromDegrees(316.877))
                        .lineTo(new Vector2(429.597, 88.203), Rotation2.fromDegrees(315))
                        .lineTo(new Vector2(426.841, 94.110), Rotation2.fromDegrees(234))
                        .build(),
                DrivebaseSubsystem.DriveConstants.TRAJECTORY_CONSTRAINTS, 0.1);

        addCommands(
                new IntakeSetExtendCommand(intakeSubsystem),
                new InstantCommand(() -> new ShooterTargetCommand(shooterSubsystem, localizer,
                        () -> false)),
                new ParallelCommandGroup(
                        new IntakeCommand(intakeSubsystem, indexSubsystem),
                        new SequentialCommandGroup(
                                new Follow2910TrajectoryCommand(drivebaseSubsystem, robotPath),
                                new ParallelCommandGroup(
                                        new IndexShootCommand(indexSubsystem, localizer),
                                        new InstantCommand(() -> new ShooterTargetCommand(shooterSubsystem, localizer,
                                                () -> true))))));
    }
}
