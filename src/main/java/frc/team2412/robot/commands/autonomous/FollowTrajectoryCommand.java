package frc.team2412.robot.commands.autonomous;

import org.frcteam2910.common.control.Trajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;

public class FollowTrajectoryCommand extends CommandBase {
    private final DrivebaseSubsystem drivebase;
    private final Trajectory trajectory;

    public FollowTrajectoryCommand(DrivebaseSubsystem drivebase, Trajectory trajectory) {
        this.drivebase = drivebase;
        this.trajectory = trajectory;

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        drivebase.getFollower().follow(trajectory);
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.getFollower().cancel();
    }

    @Override
    public boolean isFinished() {
        return drivebase.getFollower().getCurrentTrajectory().isEmpty();
    }
}
