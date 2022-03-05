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
import frc.team2412.robot.subsystem.IntakeSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.ShooterVisionSubsystem;

public class FourBallAutoCommandRight extends SequentialCommandGroup {
    public FourBallAutoCommandRight(IndexSubsystem indexSubsystem, ShooterSubsystem shooterSubsystem,
        ShooterVisionSubsystem shooterVisionSubsystem, DrivebaseSubsystem drivebaseSubsystem,
        IntakeSubsystem intakeSubsystem) {
        // robot should be in the same place as two ball auto right
        // TODO: add rotation to this
        Trajectory driveBack = new Trajectory(
            new SimplePathBuilder(new Vector2(334, 220), Rotation2.fromDegrees(68.2))
                .lineTo(new Vector2(353, 313), Rotation2.fromDegrees(68.2))
                .build(), 
            DrivebaseSubsystem.DriveConstants.TRAJECTORY_CONSTRAINTS, 0.1);
        
        Trajectory driveToTerminal = new Trajectory(
            new SimplePathBuilder(new Vector2(353, 313), Rotation2.fromDegrees(68.2))
                .lineTo(new Vector2(610, 280), Rotation2.fromDegrees(0))
                .build(), 
            DrivebaseSubsystem.DriveConstants.TRAJECTORY_CONSTRAINTS, 0.1);

        Trajectory driveToHub = new Trajectory(
            new SimplePathBuilder(new Vector2(610, 280), Rotation2.fromDegrees(0))
                .lineTo(new Vector2(452, 250), Rotation2.fromDegrees(0))
                .build(), 
            DrivebaseSubsystem.DriveConstants.TRAJECTORY_CONSTRAINTS, 0.1);

        addCommands(
            new ParallelCommandGroup(
                    new ScheduleCommand(new ShooterTargetCommand(shooterSubsystem, shooterVisionSubsystem)),
                    new WaitCommand(1)),
            new ParallelDeadlineGroup(new WaitCommand(1), new IndexShootCommand(indexSubsystem)),
            new IntakeExtendCommand(intakeSubsystem),
            new ParallelDeadlineGroup(new WaitCommand(3), new Follow2910TrajectoryCommand(drivebaseSubsystem, driveBack), new IntakeInCommand(indexSubsystem, intakeSubsystem)),
            new ParallelDeadlineGroup(new WaitCommand(4), new Follow2910TrajectoryCommand(drivebaseSubsystem, driveToTerminal), new IntakeInCommand(indexSubsystem, intakeSubsystem)),
            new ParallelCommandGroup(new Follow2910TrajectoryCommand(drivebaseSubsystem, driveToHub), new IntakeInCommand(indexSubsystem, intakeSubsystem))
        );
    }
}
