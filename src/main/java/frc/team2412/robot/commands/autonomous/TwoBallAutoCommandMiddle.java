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
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.ShooterVisionSubsystem;

public class TwoBallAutoCommandMiddle extends SequentialCommandGroup {
    /**
     * Subsystems: {@link DrivebaseSubsystem}, {@link IndexSubsystem}, {@link ShooterSubsystem},
     * {@link ShooterVisionSubsystem}
     */
    public TwoBallAutoCommandMiddle() {
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
