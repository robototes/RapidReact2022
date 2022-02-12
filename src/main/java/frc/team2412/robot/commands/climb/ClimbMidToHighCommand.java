package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.subsystem.ClimbSubsystem;

public class ClimbMidToHighCommand extends SequentialCommandGroup {

    public ClimbMidToHighCommand(ClimbSubsystem subsystem) {
        addCommands(new ExtendFixedHookCommand(subsystem),
                new RetractFixedHookCommand(subsystem));
    }

}
