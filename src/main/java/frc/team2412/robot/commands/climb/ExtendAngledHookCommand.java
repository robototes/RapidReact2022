package frc.team2412.robot.commands.climb;

import frc.team2412.robot.subsystem.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendAngledHookCommand extends CommandBase {

    private final ClimbSubsystem subsystem;

    public ExtendAngledHookCommand(ClimbSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.extendAngledArm();
    }

    @Override
    public boolean isFinished() {
        return subsystem.isDynamicFullyExtended();
    }

}
