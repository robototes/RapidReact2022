package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.subsystem.ClimbSubsystem;
import frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.AutoClimbState;

/**
 * Subsystems: {@link ClimbSubsystem}
 */
public class ClimbMidToHighCommand extends SequentialCommandGroup {
    public ClimbMidToHighCommand() {
        var subsystem = ClimbSubsystem.instance;

        subsystem.setAutoClimbState(AutoClimbState.MID_HIGH);
        addCommands(new ExtendFixedHookCommand(),
                new RetractFixedHookCommand());
    }
}
