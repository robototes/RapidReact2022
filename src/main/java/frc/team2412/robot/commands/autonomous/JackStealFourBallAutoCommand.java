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
import frc.team2412.robot.commands.intake.IntakeSetOutCommand;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.TargetLocalizer;
import frc.team2412.robot.util.UtilityCommand;

import static frc.team2412.robot.commands.autonomous.JackStealFourBallAutoCommand.StealFourBallConstants.*;

public class JackStealFourBallAutoCommand extends DynamicRequirementSequentialCommandGroup implements UtilityCommand {
    public static class StealFourBallConstants {
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
                new SimplePathBuilder(new Vector2(399.125, 133.486), Rotation2.fromDegrees(-90))
                        .arcTo(new Vector2(420.991, 87.809), new Vector2(359.901, 78.359), Rotation2.fromDegrees(-120))
                        .build(),
                NORMAL_SPEED, 0.1);

        public static final Trajectory PATH_2 = new Trajectory(
                new SimplePathBuilder(new Vector2(420.991, 87.809), Rotation2.fromDegrees(-120))
                        .lineTo(new Vector2(394.158, 48.433), Rotation2.fromDegrees(-150))
                        .build(),
                NORMAL_SPEED, 0.1);

        public static final Trajectory PATH_3 = new Trajectory(
                new SimplePathBuilder(new Vector2(394.158, 48.433), Rotation2.fromDegrees(-150))
                        .arcTo(new Vector2(456.770, 202.509), new Vector2(313.678, 161), Rotation2.fromDegrees(-270))
                        .build(),
                NORMAL_SPEED, 0.1);

        public static final Trajectory PATH_4 = new Trajectory(
                new SimplePathBuilder(new Vector2(456.770, 202.509), Rotation2.fromDegrees(-270))
                        .arcTo(new Vector2(373.468, 140.532), new Vector2(369.592, 239.367),
                                Rotation2.fromDegrees(-205))
                        .build(),
                NORMAL_SPEED, 0.1);

        public static void init() {
            System.out.println("----- 4 Ball Steal Auto Paths Initialized -----");
        }

    }

    public JackStealFourBallAutoCommand(DrivebaseSubsystem drivebaseSubsystem, IntakeSubsystem intakeSubsystem,
            IndexSubsystem indexSubsystem,
            ShooterSubsystem shooterSubsystem, TargetLocalizer localizer) {
        ShooterTargetCommand.TurretManager manager = new ShooterTargetCommand.TurretManager(shooterSubsystem,
                localizer);
        addCommands2(
                manager.scheduleCommand().alongWith(
                        new IntakeSetInCommand(intakeSubsystem),
                        new IndexSpitCommand(indexSubsystem).withTimeout(0.05)),
                manager.disableAt(0),
                new Follow2910TrajectoryCommand(drivebaseSubsystem, PATH_1),
                new IndexShootCommand(indexSubsystem).withTimeout(2),
                manager.disableAt(0),
                new Follow2910TrajectoryCommand(drivebaseSubsystem, PATH_2),
                new Follow2910TrajectoryCommand(drivebaseSubsystem, PATH_3),
                new Follow2910TrajectoryCommand(drivebaseSubsystem, PATH_4),
                new IntakeSetOutCommand(intakeSubsystem),
                new IndexSpitCommand(indexSubsystem));
    }
}
