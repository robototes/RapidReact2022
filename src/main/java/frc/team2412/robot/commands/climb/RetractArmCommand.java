package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ClimbSubsystem;

public class RetractArmCommand extends CommandBase {

    private final ClimbSubsystem subsystem;

    public RetractArmCommand(ClimbSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.retractArm();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
