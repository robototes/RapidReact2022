package frc.team2412.robot.commands.climb;

import frc.team2412.robot.subsystem.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetractFixedHookCommand extends CommandBase {

    private final ClimbSubsystem subsystem;

    public RetractFixedHookCommand(ClimbSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.retractFixedArm();
    }

    @Override
    public boolean isFinished() {
        return subsystem.isFixedFullyRetracted();
    }

}
