package frc.team2412.robot.commands.climb;

import frc.team2412.robot.subsystem.ClimbSubsystem;
import frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.AutoClimbState;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Subsystems: {@link ClimbSubsystem}
 */
public class ClimbGroundToMidCommand extends SequentialCommandGroup {
    public ClimbGroundToMidCommand() {
        var subsystem = ClimbSubsystem.instance;

        subsystem.setAutoClimbState(AutoClimbState.GROUND_MID);
        addCommands(new UnangleClimbHookCommand(),
                new ExtendAngledHookCommand(),
                new RetractAngledHookCommand());
    }
}
