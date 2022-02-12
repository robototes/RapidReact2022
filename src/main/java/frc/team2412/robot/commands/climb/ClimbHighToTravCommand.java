package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.subsystem.ClimbSubsystem;

public class ClimbHighToTravCommand extends SequentialCommandGroup {

    public ClimbHighToTravCommand(ClimbSubsystem subsystem) {
        addCommands(new AngleClimbHookCommand(subsystem),
                new ExtendAngledHookCommand(subsystem),
                new RetractAngledHookCommand(subsystem));
    }

}
