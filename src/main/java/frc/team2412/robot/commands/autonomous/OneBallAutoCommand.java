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
import frc.team2412.robot.commands.intake.IntakeSetRetractCommand;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;

public class OneBallAutoCommand extends SequentialCommandGroup {
    public OneBallAutoCommand(IndexSubsystem indexSubsystem, ShooterSubsystem shooterSubsystem,
            TargetLocalizer localizer, DrivebaseSubsystem drivebaseSubsystem, IntakeSubsystem intakeSubsystem) {
        Trajectory robotPath = new Trajectory(
                new SimplePathBuilder(Vector2.ZERO, Rotation2.ZERO)
                        .lineTo(new Vector2(0, 70))
                        .build(),
                DrivebaseSubsystem.DriveConstants.TRAJECTORY_CONSTRAINTS, 0.1);

                ShooterTargetCommand.TurretManager manager = new ShooterTargetCommand.TurretManager(shooterSubsystem,
                localizer);

        addCommands(

                manager.scheduleCommand(),
                manager.disableAt(0),
                new IndexShootCommand(indexSubsystem, localizer).withTimeout(4),
                new Follow2910TrajectoryCommand(drivebaseSubsystem, robotPath)
        );
    }
}
