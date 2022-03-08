package frc.team2412.robot.commands.autonomous;

import frc.team2412.robot.subsystem.*;
import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.commands.index.IndexShootCommand;
import frc.team2412.robot.commands.intake.IntakeExtendCommand;
import frc.team2412.robot.commands.intake.IntakeInCommand;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;

public class TwoBallAutoCommandMiddle extends SequentialCommandGroup {
    public TwoBallAutoCommandMiddle(IndexSubsystem indexSubsystem, ShooterSubsystem shooterSubsystem,
            TargetLocalizer localizer, DrivebaseSubsystem drivebaseSubsystem,
            IntakeSubsystem intakeSubsystem) {
        // Robot should be pressed up on the right side of the lower exit closest to the drivers, facing
        // directly away from the hub with the turret facing towards it
        Trajectory robotPath = new Trajectory(
                new SimplePathBuilder(new Vector2(359, 209), Rotation2.fromDegrees(68.2))
                        .lineTo(new Vector2(445, 247.7), Rotation2.fromDegrees(15.4))
                        .build(),
                DrivebaseSubsystem.DriveConstants.TRAJECTORY_CONSTRAINTS, 0.1);

        // This shoots, drives back to where the ball is with the intake extended, picks it up and
        // immediately shoots it
        addCommands(
                new ParallelCommandGroup(
                        new ScheduleCommand(new ShooterTargetCommand(shooterSubsystem, localizer)),
                        new WaitCommand(1)),
                new ParallelDeadlineGroup(new WaitCommand(1), new IndexShootCommand(indexSubsystem)),
                new IntakeExtendCommand(intakeSubsystem),
                new ParallelCommandGroup(
                        new Follow2910TrajectoryCommand(drivebaseSubsystem, robotPath),
                        new IntakeInCommand(indexSubsystem, intakeSubsystem)));

    }
}
