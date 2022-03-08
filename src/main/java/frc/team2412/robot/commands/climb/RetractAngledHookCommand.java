package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team2412.robot.subsystem.ClimbSubsystem;

public class RetractAngledHookCommand extends CommandBase {
    private final ClimbSubsystem subsystem;

    /**
     * Subsystems: {@link ClimbSubsystem}
     */
    public RetractAngledHookCommand() {
        this.subsystem = ClimbSubsystem.instance;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.retractAngledArm();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopFixedArm();
    }
}
