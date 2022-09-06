package frc.team2412.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.team2412.robot.commands.index.IndexShootCommand;
import frc.team2412.robot.commands.index.IndexSpitCommand;
import frc.team2412.robot.commands.intake.IntakeSetInCommand;
import frc.team2412.robot.commands.intake.IntakeSetStopCommand;
import org.frcteam2910.common.control.CentripetalAccelerationConstraint;
import org.frcteam2910.common.control.FeedforwardConstraint;
import org.frcteam2910.common.control.MaxAccelerationConstraint;
import org.frcteam2910.common.control.MaxVelocityConstraint;
import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.control.TrajectoryConstraint;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import frc.team2412.robot.commands.shooter.ShooterTargetCommand;
import frc.team2412.robot.subsystem.Constants;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.TargetLocalizer;
import frc.team2412.robot.util.UtilityCommand;

import static frc.team2412.robot.commands.autonomous.JackFiveBallAutoCommand.FiveBallConstants.*;

public class JackFiveBallAutoCommand extends SequentialCommandGroup implements UtilityCommand {
    public static class FiveBallConstants {
        public static final TrajectoryConstraint[] NORMAL_SPEED = {
                new FeedforwardConstraint(11.0,
                        Constants.DriveConstants.FEEDFORWARD_CONSTANTS.getVelocityConstant(),
                        Constants.DriveConstants.FEEDFORWARD_CONSTANTS.getAccelerationConstant(), false), // old
                // value
                // was
                // 11.0
                new MaxAccelerationConstraint(9 * 12.0), // old value was 12.5 * 12.0
                new MaxVelocityConstraint(11.5 * 12.0),
                new CentripetalAccelerationConstraint(6 * 12.0), // old value was 15 * 12.0
        };

        public static final TrajectoryConstraint[] FAST_SPEED = {
                new FeedforwardConstraint(11.0,
                        Constants.DriveConstants.FEEDFORWARD_CONSTANTS.getVelocityConstant(),
                        Constants.DriveConstants.FEEDFORWARD_CONSTANTS.getAccelerationConstant(), false), // old
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

        public static final Trajectory PATH_1 = new Trajectory(
                new SimplePathBuilder(new Vector2(328, 75.551), Rotation2.fromDegrees(-90))
                        .arcTo(new Vector2(293.811, 25.771), new Vector2(281.299, 92.401), Rotation2.fromDegrees(-90))
                        .lineTo(new Vector2(218.203, 60.492), Rotation2.fromDegrees(130))
                        .build(),
                NORMAL_SPEED, 0.1);

        public static final Trajectory PATH_2 = new Trajectory(
                new SimplePathBuilder(new Vector2(213.203, 66.492), Rotation2.fromDegrees(130))
                        .lineTo(new Vector2(195.049, 82.693), Rotation2.fromDegrees(125))
                        .build(),
                FAST_SPEED, 0.1);

        public static final Trajectory PATH_3 = new Trajectory(
                new SimplePathBuilder(new Vector2(195.029, 75.188), Rotation2.fromDegrees(125))
                        .lineTo(new Vector2(50.456, 85), Rotation2.fromDegrees(202))
                        .build(),
                NORMAL_SPEED, 0.1);

        public static final Trajectory PATH_4 = new Trajectory(
                new SimplePathBuilder(new Vector2(50.456, 85), Rotation2.fromDegrees(202))
                        .lineTo(new Vector2(56.456, 91), Rotation2.fromDegrees(202))
                        .build(),
                NORMAL_SPEED, 0.1);

        // public static final Trajectory PATH_3 = new Trajectory(
        // new SimplePathBuilder(new Vector2(195.029, 75.188), Rotation2.fromDegrees(125))
        // .lineTo(new Vector2(94.653, 36.976), Rotation2.fromDegrees(-180))
        // .arcTo(new Vector2(22.456, 89.547), new Vector2(130, 103))
        // .build(),
        // NORMAL_SPEED, 0.1);

        public static final Trajectory PATH_5 = new Trajectory(
                new SimplePathBuilder(new Vector2(56.456, 91), Rotation2.fromDegrees(202))
                        .lineTo(new Vector2(207.029, 82.188), Rotation2.fromDegrees(202))
                        .build(),
                NORMAL_SPEED, 0.1);

        // public static final Trajectory PATH_4 = new Trajectory(
        // new SimplePathBuilder(new Vector2(22.456, 89.547), Rotation2.fromDegrees(-180))
        // .lineTo(new Vector2(207.029, 82.188), Rotation2.fromDegrees(202))
        // .build(),
        // NORMAL_SPEED, 0.1);

        public static void init() {
            System.out.println("----- 5 Ball Auto Paths Initialized -----");
        }

    }

    public JackFiveBallAutoCommand(DrivebaseSubsystem drivebaseSubsystem, IntakeSubsystem intakeSubsystem,
            IndexSubsystem indexSubsystem,
            ShooterSubsystem shooterSubsystem, TargetLocalizer localizer) {

        ShooterTargetCommand.TurretManager manager = new ShooterTargetCommand.TurretManager(shooterSubsystem,
                localizer);

        // indexSubsystem.setDefaultCommand(new IndexCommand(indexSubsystem, intakeSubsystem));
        addCommands(
                manager.scheduleCommand().alongWith(
                        new IntakeSetInCommand(intakeSubsystem),
                        new IndexSpitCommand(indexSubsystem).withTimeout(0.05)),
                manager.disableAt(-13),
                new Follow2910TrajectoryCommand(drivebaseSubsystem, PATH_1),
                manager.enableAt(-13),
                await(1),
                new IndexShootCommand(indexSubsystem).withTimeout(2),
                new Follow2910TrajectoryCommand(drivebaseSubsystem, PATH_2),
                new IndexShootCommand(indexSubsystem).withTimeout(1),
                manager.disableAt(70),
                new Follow2910TrajectoryCommand(drivebaseSubsystem, PATH_3),
                new Follow2910TrajectoryCommand(drivebaseSubsystem, PATH_4),
                await(1),
                new Follow2910TrajectoryCommand(drivebaseSubsystem, PATH_5),
                manager.enableAt(70),
                new IndexShootCommand(indexSubsystem).withTimeout(2),
                new ParallelCommandGroup(
                        manager.disableAt(0),
                        new IntakeSetStopCommand(intakeSubsystem)));

    }

}
