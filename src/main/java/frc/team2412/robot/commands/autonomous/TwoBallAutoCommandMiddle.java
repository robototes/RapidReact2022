package frc.team2412.robot.commands.autonomous;

import frc.team2412.robot.commands.index.IndexShootCommand;
import frc.team2412.robot.commands.index.IndexSpitCommand;
import frc.team2412.robot.commands.intake.IntakeSetInCommand;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;
import frc.team2412.robot.subsystem.*;
import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TwoBallAutoCommandMiddle extends SequentialCommandGroup {
    public TwoBallAutoCommandMiddle(IndexSubsystem indexSubsystem, ShooterSubsystem shooterSubsystem,
            TargetLocalizer localizer, DrivebaseSubsystem drivebaseSubsystem,
            IntakeSubsystem intakeSubsystem) {
        Trajectory robotPath = new Trajectory(
                new SimplePathBuilder(new Vector2(381.791, 211.487), Rotation2.fromDegrees(25))
                        .lineTo(new Vector2(426.405, 210.657), Rotation2.fromDegrees(25))
                        .lineTo(new Vector2(420, 215), Rotation2.fromDegrees(-65))
                        .build(),
                DrivebaseSubsystem.DriveConstants.TRAJECTORY_CONSTRAINTS, 0.1);

        ShooterTargetCommand.TurretManager manager = new ShooterTargetCommand.TurretManager(shooterSubsystem,
                localizer);

        addCommands(

                manager.scheduleCommand().alongWith(
                        new IntakeSetInCommand(intakeSubsystem),
                        new WaitCommand(2),
                        new IndexSpitCommand(indexSubsystem).withTimeout(0.05)),

                manager.disableAt(-20),
                new Follow2910TrajectoryCommand(drivebaseSubsystem, robotPath),
                manager.enableAt(-20),
                new IndexShootCommand(indexSubsystem, localizer).withTimeout(4));
    }
}
