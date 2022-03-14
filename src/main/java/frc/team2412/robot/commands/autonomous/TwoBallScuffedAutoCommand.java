package frc.team2412.robot.commands.autonomous;

import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.commands.index.IndexShootCommand;
import frc.team2412.robot.commands.intake.IntakeIndexInCommand;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.ShooterVisionSubsystem;

public class TwoBallScuffedAutoCommand extends SequentialCommandGroup {
    public TwoBallScuffedAutoCommand(IndexSubsystem indexSubsystem, ShooterSubsystem shooterSubsystem,
            ShooterVisionSubsystem shooterVisionSubsystem, DrivebaseSubsystem drivebaseSubsystem,
            IntakeSubsystem intakeSubsystem) {
        Trajectory robotPath = new Trajectory(
                new SimplePathBuilder(Vector2.ZERO, Rotation2.ZERO)
                        .lineTo(new Vector2(0, 70), Rotation2.fromDegrees(90))
                        .lineTo(new Vector2(0, 60), Rotation2.ZERO)
                        .build(),
                DrivebaseSubsystem.DriveConstants.TRAJECTORY_CONSTRAINTS, 0.1);

        addCommands(
                new ParallelCommandGroup(
                        // new ShooterTargetCommand(shooterSubsystem, shooterVisionSubsystem),
                        new ScheduleCommand(new ShooterTargetCommand(shooterSubsystem, shooterVisionSubsystem)),
                        new WaitCommand(1)),
                new IndexShootCommand(indexSubsystem).withTimeout(1),
                new Follow2910TrajectoryCommand(drivebaseSubsystem, robotPath)
                        .alongWith(new IntakeIndexInCommand(indexSubsystem, intakeSubsystem)),
                new IndexShootCommand(indexSubsystem).withTimeout(1));

    }
}
