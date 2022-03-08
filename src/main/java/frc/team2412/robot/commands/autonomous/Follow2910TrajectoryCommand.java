package frc.team2412.robot.commands.autonomous;

import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.control.Path.State;
import org.frcteam2910.common.math.RigidTransform2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;

public class Follow2910TrajectoryCommand extends CommandBase {
    private final DrivebaseSubsystem drivebase;
    private final Trajectory trajectory;

    /**
     * Subsystems: {@link DrivebaseSubsystem}
     */
    public Follow2910TrajectoryCommand(Trajectory trajectory) {
        this.drivebase = DrivebaseSubsystem.instance;
        this.trajectory = trajectory;

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        State pathStartState = trajectory.getPath().calculate(0);
        drivebase.resetPose(new RigidTransform2(pathStartState.getPosition(), pathStartState.getHeading()));
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
