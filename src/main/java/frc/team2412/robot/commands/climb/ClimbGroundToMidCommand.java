package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.team2412.robot.subsystem.ClimbSubsystem;
import frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.AutoClimbState;

public class ClimbGroundToMidCommand extends SequentialCommandGroup {
    /**
     * Subsystems: {@link ClimbSubsystem}
     */
    public ClimbGroundToMidCommand() {
        var subsystem = ClimbSubsystem.instance;

        subsystem.setAutoClimbState(AutoClimbState.GROUND_MID);
        addCommands(new UnangleClimbHookCommand(),
                new ExtendAngledHookCommand(),
                new RetractAngledHookCommand());
    }
}
