package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team2412.robot.subsystem.ClimbSubsystem;

public class ExtendAngledHookCommand extends CommandBase {
    private final ClimbSubsystem subsystem;

    /**
     * Subsystems: {@link ClimbSubsystem}
     */
    public ExtendAngledHookCommand() {
        this.subsystem = ClimbSubsystem.instance;
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
