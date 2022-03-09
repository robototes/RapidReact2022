package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team2412.robot.subsystem.ClimbSubsystem;

public class NeutralClimbHookCommand extends CommandBase {
    private final ClimbSubsystem subsystem;

    /**
     * Subsystems: {@link ClimbSubsystem} Sets the climb angled hook to the neutral position.
     */
    public NeutralClimbHookCommand() {
        this.subsystem = ClimbSubsystem.instance;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.angleClimbHook(DoubleSolenoid.Value.kOff);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
