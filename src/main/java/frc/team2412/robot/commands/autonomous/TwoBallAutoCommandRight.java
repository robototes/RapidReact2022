package frc.team2412.robot.commands.autonomous;

import frc.team2412.robot.commands.intake.IntakeSetExtendCommand;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;
import frc.team2412.robot.commands.intake.IntakeCommand;
import frc.team2412.robot.commands.index.IndexShootCommand;
import frc.team2412.robot.subsystem.*;
import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TwoBallAutoCommandRight extends SequentialCommandGroup {
    public TwoBallAutoCommandRight(IndexSubsystem indexSubsystem, ShooterSubsystem shooterSubsystem,
            TargetLocalizer localizer, DrivebaseSubsystem drivebaseSubsystem,
            IntakeSubsystem intakeSubsystem) {
        Trajectory robotPath = new Trajectory(
                new SimplePathBuilder(new Vector2(341, 250.434), Rotation2.fromDegrees(90))
                        .lineTo(new Vector2(337.850, 290.717), Rotation2.fromDegrees(90))
                        .lineTo(new Vector2(337.850, 287), Rotation2.fromDegrees(0))
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
                                new WaitCommand(1),
                                new ParallelCommandGroup(
                                        new IndexShootCommand(indexSubsystem, localizer),
                                        new InstantCommand(() -> new ShooterTargetCommand(shooterSubsystem, localizer,
                                                () -> true))))));
    }
}
