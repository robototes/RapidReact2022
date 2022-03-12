package frc.team2412.robot.commands.climb;

import frc.team2412.robot.subsystem.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FullExtendFixedHookCommand extends CommandBase {

    private final ClimbSubsystem subsystem;

    public FullExtendFixedHookCommand(ClimbSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.extendArm();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
