package frc.team2412.robot.commands.autonomous;

import java.util.List;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.commands.index.IndexShootCommand;
import frc.team2412.robot.commands.intake.IntakeCommand;
import frc.team2412.robot.commands.intake.IntakeSetExtendCommand;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.TargetLocalizer;

public class WPILibFiveBallAutoCommand extends SequentialCommandGroup {
    public WPILibFiveBallAutoCommand(DrivebaseSubsystem drivebaseSubsystem, IntakeSubsystem intakeSubsystem,
            IndexSubsystem indexSubsystem, ShooterSubsystem shooterSubsystem, TargetLocalizer localizer) {

        TrajectoryConfig normalSpeedConfig = new TrajectoryConfig(1, 1)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(FollowWpilibTrajectory.WPILibAutoConstants.driveKinematics);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                FollowWpilibTrajectory.WPILibAutoConstants.DEFAULT_THETA, 0, 0,
                FollowWpilibTrajectory.WPILibAutoConstants.K_THETA_CONTROLLER_CONSTRAINTS);

        Trajectory trajectoryOne = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(8.4, 1.8, Rotation2d.fromDegrees(-90)),
                        new Pose2d(7.4, 0.9, Rotation2d.fromDegrees(180)),
                        new Pose2d(5.3, 1.8, Rotation2d.fromDegrees(180))),
                normalSpeedConfig);
        Trajectory trajectoryTwo = TrajectoryGenerator.generateTrajectory(
                new Pose2d(5.3, 1.8, Rotation2d.fromDegrees(180)),
                List.of(),
                new Pose2d(2.0, 1.3, Rotation2d.fromDegrees(180)), normalSpeedConfig);
        Trajectory trajectoryThree = TrajectoryGenerator.generateTrajectory(
                new Pose2d(2.0, 1.3, Rotation2d.fromDegrees(0)),
                List.of(),
                new Pose2d(5, 2.7, Rotation2d.fromDegrees(0)), normalSpeedConfig);

        addCommands(
                new ParallelCommandGroup(
                        // paths
                        new SequentialCommandGroup(
                                new FollowWpilibTrajectory(drivebaseSubsystem, trajectoryOne, thetaController),
                                new FollowWpilibTrajectory(drivebaseSubsystem, trajectoryTwo, thetaController),
                                new FollowWpilibTrajectory(drivebaseSubsystem, trajectoryThree, thetaController))),

                // actions
                // STEPS FOR COMMAND
                // 1. drive to ball 2, intake,
                // 2. drive to ball 3, intake, shoot
                // 3. drive to ball 4, intake
                // 4. wait 2ish seconds, intake,
                // 5. drive, shoot

                new SequentialCommandGroup(
                        new IntakeSetExtendCommand(intakeSubsystem), new WaitCommand(2),
                        new IntakeCommand(intakeSubsystem, indexSubsystem),
                        new WaitCommand(3), new IntakeCommand(intakeSubsystem, indexSubsystem),
                        // new IntakeIndexInCommand(indexSubsystem, intakeSubsystem),
                        new ScheduleCommand(new ShooterTargetCommand(shooterSubsystem, localizer)),
                        new WaitCommand(3), new IntakeCommand(intakeSubsystem, indexSubsystem),
                        new WaitCommand(2), new IntakeCommand(intakeSubsystem, indexSubsystem),
                        new WaitCommand(3), new ScheduleCommand(new ShooterTargetCommand(shooterSubsystem, localizer)),
                        new ParallelDeadlineGroup(new WaitCommand(1), new IndexShootCommand(indexSubsystem))));
    }

}
