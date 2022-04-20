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

import frc.team2412.robot.commands.index.IndexShootCommand;
import frc.team2412.robot.commands.index.IndexSpitCommand;
import frc.team2412.robot.commands.intake.IntakeSetInCommand;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.TargetLocalizer;
import frc.team2412.robot.util.UtilityCommand;

import static frc.team2412.robot.commands.autonomous.JackStealThreeBallAutoCommand.StealThreeBallConstants.*;

public class JackStealThreeBallAutoCommand extends DynamicRequirementSequentialCommandGroup implements UtilityCommand {
    public static class StealThreeBallConstants {
        public static final TrajectoryConstraint[] NORMAL_SPEED = {
                new FeedforwardConstraint(11.0,
                        DrivebaseSubsystem.DriveConstants.FEEDFORWARD_CONSTANTS.getVelocityConstant(),
                        DrivebaseSubsystem.DriveConstants.FEEDFORWARD_CONSTANTS.getAccelerationConstant(), false), // old
                // value
                // was
                // 11.0
                new MaxAccelerationConstraint(9 * 12.0), // old value was 12.5 * 12.0
                new MaxVelocityConstraint(11.5 * 12.0),
                new CentripetalAccelerationConstraint(6 * 12.0), // old value was 15 * 12.0
        };

        public static final Trajectory PATH_1 = new Trajectory(
                new SimplePathBuilder(new Vector2(401.398, 177.473), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(448.256, 191.255), Rotation2.fromDegrees(0))
                        .arcTo(new Vector2(551, 70), new Vector2(447, 114), Rotation2.fromDegrees(180))
                        .lineTo(new Vector2(440, 16))
                        .lineTo(new Vector2(383.917, 38.712))
                        .build(),
                NORMAL_SPEED, 0.1);

        public static final Trajectory PATH_2 = new Trajectory(
                new SimplePathBuilder(new Vector2(393.917, 48.712), Rotation2.fromDegrees(180))
                        .lineTo(new Vector2(372.653, 138.097), Rotation2.fromDegrees(150))
                        .build(),
                NORMAL_SPEED, 0.1);

        public static void init() {
            System.out.println("----- 3 Ball Steal Auto Paths Initialized -----");
        }

    }

    public JackStealThreeBallAutoCommand(DrivebaseSubsystem drivebaseSubsystem, IntakeSubsystem intakeSubsystem,
            IndexSubsystem indexSubsystem,
            ShooterSubsystem shooterSubsystem, TargetLocalizer localizer) {
        ShooterTargetCommand.TurretManager manager = new ShooterTargetCommand.TurretManager(shooterSubsystem,
                localizer);
        addCommands2(
                manager.scheduleCommand().alongWith(
                        new IntakeSetInCommand(intakeSubsystem),
                        new IndexSpitCommand(indexSubsystem).withTimeout(0.05)),
                manager.enableAt(190),
                new IndexShootCommand(indexSubsystem).withTimeout(2),
                manager.disableAt(0),
                new Follow2910TrajectoryCommand(drivebaseSubsystem, PATH_1),
                new Follow2910TrajectoryCommand(drivebaseSubsystem, PATH_2),
                new IndexSpitCommand(indexSubsystem));
    }
}
