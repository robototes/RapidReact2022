package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team2412.robot.subsystem.ClimbSubsystem;

public class FullExtendFixedHookCommand extends CommandBase {
    private final ClimbSubsystem subsystem;

    /**
     * Subsystems: {@link ClimbSubsystem}
     */
    public FullExtendFixedHookCommand() {
        this.subsystem = ClimbSubsystem.instance;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.extendFixedArm();
    }

    // @Override
    // public boolean isFinished() {
    // return subsystem.isFixedFullyExtended();
    // }

    // @Override
    // public void end(boolean interrupted) {
    // subsystem.stopFixedArm();
    // }
}
