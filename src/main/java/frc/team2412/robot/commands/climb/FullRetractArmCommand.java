package frc.team2412.robot.commands.climb;

import frc.team2412.robot.subsystem.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FullRetractArmCommand extends CommandBase {

    private final ClimbSubsystem subsystem;

    public FullRetractArmCommand(ClimbSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.retractArmFully();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
