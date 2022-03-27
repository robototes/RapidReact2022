package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.subsystem.ClimbSubsystem;

/**
 * Extend and then retract the climb arm to verify that
 * climb motors are working correctly
 */
public class ClimbTestCommand extends SequentialCommandGroup {

    public ClimbTestCommand(ClimbSubsystem subsystem) {
        addCommands(
                new ExtendArmCommand(subsystem), new WaitCommand(1),
                new RetractArmCommand(subsystem));
    }

}
