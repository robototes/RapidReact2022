package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.PostClimbSubsystem;

public class PostClimbUpComamnd extends CommandBase {

    private PostClimbSubsystem subsystem;

    public PostClimbUpComamnd(PostClimbSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.armSolenoid();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
