package frc.team2412.robot.commands.autonomous;

import org.frcteam2910.common.control.CentripetalAccelerationConstraint;
import org.frcteam2910.common.control.FeedforwardConstraint;
import org.frcteam2910.common.control.MaxAccelerationConstraint;
import org.frcteam2910.common.control.MaxVelocityConstraint;
import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.control.TrajectoryConstraint;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.commands.index.IndexShootCommand;
import frc.team2412.robot.commands.intake.IntakeCommand;
import frc.team2412.robot.commands.intake.IntakeSetExtendCommand;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.TargetLocalizer;

public class JackFiveBallAutoCommand extends SequentialCommandGroup {
    public JackFiveBallAutoCommand(DrivebaseSubsystem drivebaseSubsystem, IntakeSubsystem intakeSubsystem,
            IndexSubsystem indexSubsystem,
            ShooterSubsystem shooterSubsystem, TargetLocalizer localizer) {

        TrajectoryConstraint[] normalSpeed = {
                new FeedforwardConstraint(11.0,
                        DrivebaseSubsystem.DriveConstants.FEEDFORWARD_CONSTANTS.getVelocityConstant(),
                        DrivebaseSubsystem.DriveConstants.FEEDFORWARD_CONSTANTS.getAccelerationConstant(), false), // old
                                                                                                                    // value
                                                                                                                    // was
                                                                                                                    // 11.0
                new MaxAccelerationConstraint(6 * 12.0), // old value was 12.5 * 12.0
                new MaxVelocityConstraint(8 * 12.0),
                new CentripetalAccelerationConstraint(6 * 12.0), // old value was 15 * 12.0
        };

        TrajectoryConstraint[] fastSpeed = {
                new FeedforwardConstraint(11.0,
                        DrivebaseSubsystem.DriveConstants.FEEDFORWARD_CONSTANTS.getVelocityConstant(),
                        DrivebaseSubsystem.DriveConstants.FEEDFORWARD_CONSTANTS.getAccelerationConstant(), false), // old
                                                                                                                    // value
                                                                                                                    // was
                                                                                                                    // 11.0
                new MaxAccelerationConstraint(12.5 * 12.0), // old value was 12.5 * 12.0
                new MaxVelocityConstraint(4 * 12.0),
                new CentripetalAccelerationConstraint(6 * 12.0), // old value was 15 * 12.0
        };

        // go down to first ball, arc down to pick it up
        // go to second ball, stop and shoot imediately
        // then pick up the second ball and shoot it
        // move to third ball, stop and intake for collection
        // move to where second ball was and shoot

        Trajectory trajectory1 = new Trajectory(
                new SimplePathBuilder(new Vector2(328, 75.551), Rotation2.fromDegrees(-90))
                        .arcTo(new Vector2(293.811, 29.771), new Vector2(281.299, 92.401), Rotation2.fromDegrees(-90))
                        .lineTo(new Vector2(218.203, 60.492), Rotation2.fromDegrees(130))
                        .build(),
                normalSpeed, 0.1);

        Trajectory trajectory2 = new Trajectory(
                new SimplePathBuilder(new Vector2(213.203, 66.492), Rotation2.fromDegrees(130))
                        .lineTo(new Vector2(202.049, 82.693), Rotation2.fromDegrees(125))
                        .build(),
                fastSpeed, 0.1);

        Trajectory trajectory3 = new Trajectory(
                new SimplePathBuilder(new Vector2(202.029, 75.188), Rotation2.fromDegrees(125))
                        .lineTo(new Vector2(50.456, 60.818), Rotation2.fromDegrees(202))
                        .build(),
                normalSpeed, 0.1);

        Trajectory trajectory4 = new Trajectory(
                new SimplePathBuilder(new Vector2(50.456, 54.818), Rotation2.fromDegrees(202))
                        .lineTo(new Vector2(207.029, 82.188), Rotation2.fromDegrees(125))
                        .build(),
                normalSpeed, 0.1);

        addCommands(
                new IntakeSetExtendCommand(intakeSubsystem),
                new ParallelCommandGroup(
                        new IntakeCommand(intakeSubsystem, indexSubsystem),
                        new SequentialCommandGroup(
                                //new ScheduleCommand(new ShooterTargetCommand(shooterSubsystem, localizer)),
                                new InstantCommand(() -> {shooterSubsystem.setTurretDisable(true);}),
                                new Follow2910TrajectoryCommand(drivebaseSubsystem, trajectory1),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(1),
                                        new IndexShootCommand(indexSubsystem, localizer),
                                        new ShooterTargetCommand(shooterSubsystem, localizer, ()->true)),
                                new Follow2910TrajectoryCommand(drivebaseSubsystem, trajectory2),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(1),
                                        new IndexShootCommand(indexSubsystem, localizer)),
                                new InstantCommand(()->new ShooterTargetCommand(shooterSubsystem, localizer, ()->false)),
                                new Follow2910TrajectoryCommand(drivebaseSubsystem, trajectory3),
                                new WaitCommand(2),
                                new Follow2910TrajectoryCommand(drivebaseSubsystem, trajectory4),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(3),
                                        new IndexShootCommand(indexSubsystem, localizer),
                                        new ShooterTargetCommand(shooterSubsystem, localizer, ()->true)))));
    }
}
