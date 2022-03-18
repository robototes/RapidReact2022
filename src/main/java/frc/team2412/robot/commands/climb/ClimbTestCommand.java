package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.subsystem.ClimbSubsystem;

public class ClimbTestCommand extends SequentialCommandGroup {

    public ClimbTestCommand(ClimbSubsystem subsystem) {
        addCommands(
                new ExtendArmCommand(subsystem), new WaitCommand(0.5),
                new RetractArmCommand(subsystem), new WaitCommand(0.5),
                new RetractArmFullyCommand(subsystem));
    }

}
