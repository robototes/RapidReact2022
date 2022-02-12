package frc.team2412.robot.commands.climb;

import frc.team2412.robot.subsystem.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ClimbGroundToMidCommand extends SequentialCommandGroup {

    public ClimbGroundToMidCommand(ClimbSubsystem subsystem) {
        addCommands(new UnangleClimbHookCommand(subsystem),
                new ExtendAngledHookCommand(subsystem),
                new RetractAngledHookCommand(subsystem));
    }

}
