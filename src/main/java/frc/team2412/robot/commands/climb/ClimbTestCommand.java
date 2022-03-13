package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.subsystem.ClimbSubsystem;

public class ClimbTestCommand extends SequentialCommandGroup {

    public ClimbTestCommand(ClimbSubsystem subsystem) {
        addCommands(new ExtendArmCommand(subsystem),
                new RetractArmCommand(subsystem));
    }

}
