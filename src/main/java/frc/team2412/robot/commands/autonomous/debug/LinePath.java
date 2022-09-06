package frc.team2412.robot.commands.autonomous.debug;

import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.commands.autonomous.Follow2910TrajectoryCommand;
import frc.team2412.robot.subsystem.Constants;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;

public class LinePath extends SequentialCommandGroup {
    public LinePath(DrivebaseSubsystem drivebaseSubsystem) {
        Trajectory linePathAuto = new Trajectory(
                new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO)
                        .lineTo(new Vector2(24, 0))
                        .build(),
                Constants.DriveConstants.TRAJECTORY_CONSTRAINTS, 0.1);

        addCommands(new Follow2910TrajectoryCommand(drivebaseSubsystem, linePathAuto));
    }
}
