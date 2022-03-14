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
import frc.team2412.robot.commands.intake.IntakeSetExtendCommand;
import frc.team2412.robot.commands.intake.IntakeIndexInCommand;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.ShooterVisionSubsystem;

public class TwoBallAutoCommandRight extends SequentialCommandGroup {
    public TwoBallAutoCommandRight(IndexSubsystem indexSubsystem, ShooterSubsystem shooterSubsystem,
            ShooterVisionSubsystem shooterVisionSubsystem, DrivebaseSubsystem drivebaseSubsystem,
            IntakeSubsystem intakeSubsystem) {
        // Robot should be pressed up on the left side of the lower exit further from the drivers on their
        // right, facing directly away from the hub with the turret facing towards it
        Trajectory robotPath = new Trajectory(
                new SimplePathBuilder(new Vector2(334, 220), Rotation2.fromDegrees(68.2))
                        .lineTo(new Vector2(353, 308), Rotation2.fromDegrees(62))
                        .build(),
                DrivebaseSubsystem.DriveConstants.TRAJECTORY_CONSTRAINTS, 0.1);

        // This shoots, drives back to where the ball is with the intake extended, picks it up and
        // immediately shoots it
        addCommands(
                new ParallelCommandGroup(
                        new ScheduleCommand(new ShooterTargetCommand(shooterSubsystem, shooterVisionSubsystem)),
                        new WaitCommand(1)),
                new ParallelDeadlineGroup(new WaitCommand(1), new IndexShootCommand(indexSubsystem)),
                new IntakeSetExtendCommand(intakeSubsystem),
                new ParallelCommandGroup(
                        new Follow2910TrajectoryCommand(drivebaseSubsystem, robotPath),
                        new IntakeIndexInCommand(indexSubsystem, intakeSubsystem)));

    }
}
