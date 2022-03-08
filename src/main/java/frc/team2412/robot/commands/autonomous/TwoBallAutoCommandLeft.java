package frc.team2412.robot.commands.autonomous;

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
import frc.team2412.robot.subsystem.DrivebaseSubsystem;

public class TwoBallAutoCommandLeft extends SequentialCommandGroup {
    public TwoBallAutoCommandLeft() {
        // Robot should be pressed up on the left side of the lower exit closest to the drivers, facing
        // directly away from the hub with the turret facing towards it
        Trajectory robotPath = new Trajectory(
                new SimplePathBuilder(new Vector2(337, 133), Rotation2.fromDegrees(-26.4))
                        .lineTo(new Vector2(449, 84), Rotation2.fromDegrees(26.7))
                        .build(),
                DrivebaseSubsystem.DriveConstants.TRAJECTORY_CONSTRAINTS, 0.1);

        // This shoots, drives back to where the ball is with the intake extended, picks it up and
        // immediately shoots it
        addCommands(
                new ParallelCommandGroup(
                        new ScheduleCommand(
                                new ShooterTargetCommand()),
                        new WaitCommand(1)),
                new ParallelDeadlineGroup(new WaitCommand(1), new IndexShootCommand()),
                new IntakeExtendCommand(),
                new ParallelCommandGroup(
                        new Follow2910TrajectoryCommand(robotPath),
                        new IntakeInCommand()));

    }
}
