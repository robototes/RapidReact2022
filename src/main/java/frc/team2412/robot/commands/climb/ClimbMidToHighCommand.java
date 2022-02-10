package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ClimbSubsystem;

public class ClimbMidToHighCommand extends CommandBase {

    ClimbSubsystem subsystem;

    public ClimbMidToHighCommand(ClimbSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        new ExtendFixedHookCommand(subsystem)
                .andThen(new RetractFixedHookCommand(subsystem));
    }

}
