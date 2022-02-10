package frc.team2412.robot.commands.climb;

import frc.team2412.robot.subsystem.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbGroundToMidCommand extends CommandBase {

    private final ClimbSubsystem subsystem;

    public ClimbGroundToMidCommand(ClimbSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        new UnangleClimbHookCommand(subsystem)
                .andThen(new ExtendAngledHookCommand(subsystem))
                .andThen(new RetractAngledHookCommand(subsystem));
    }

    @Override
    public boolean isFinished() {
        return subsystem.isDynamicFullyExtended();
    }

}
