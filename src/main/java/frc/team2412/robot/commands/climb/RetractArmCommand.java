package frc.team2412.robot.commands.climb;

import frc.team2412.robot.subsystem.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

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
