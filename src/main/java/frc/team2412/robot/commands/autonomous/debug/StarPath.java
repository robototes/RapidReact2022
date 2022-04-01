package frc.team2412.robot.commands.autonomous.debug;

import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.commands.autonomous.Follow2910TrajectoryCommand;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;

public class StarPath extends SequentialCommandGroup {
    public StarPath(DrivebaseSubsystem drivebaseSubsystem) {
        Trajectory starPathAuto = new Trajectory(
                new SimplePathBuilder(new Vector2(12, 0), Rotation2.ZERO)
                        .lineTo(new Vector2(24, 12))
                        .lineTo(new Vector2(36, 0))
                        .lineTo(new Vector2(30, 18))
                        .lineTo(new Vector2(42, 30))
                        .lineTo(new Vector2(30, 30))
                        .lineTo(new Vector2(24, 42), Rotation2.fromDegrees(90))
                        .lineTo(new Vector2(18, 30), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(6, 30))
                        .lineTo(new Vector2(18, 18))
                        .lineTo(new Vector2(12, 0))
                        .build(),
                DrivebaseSubsystem.DriveConstants.TRAJECTORY_CONSTRAINTS, 0.1);

        addCommands(new Follow2910TrajectoryCommand(drivebaseSubsystem, starPathAuto));
    }
}
