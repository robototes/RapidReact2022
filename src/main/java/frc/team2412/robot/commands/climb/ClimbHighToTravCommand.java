package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.team2412.robot.subsystem.ClimbSubsystem;
import frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.AutoClimbState;

public class ClimbHighToTravCommand extends SequentialCommandGroup {
    /**
     * Subsystems: {@link ClimbSubsystem}
     */
    public ClimbHighToTravCommand() {
        var subsystem = ClimbSubsystem.instance;

        subsystem.setAutoClimbState(AutoClimbState.HIGH_TRAV);
        addCommands(new AngleClimbHookCommand(), new ExtendAngledHookCommand(), new RetractAngledHookCommand());
    }
}
