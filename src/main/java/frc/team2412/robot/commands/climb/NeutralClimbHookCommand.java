package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.team2412.robot.subsystem.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

// Set the climb angled hook to neutral position
public class NeutralClimbHookCommand extends CommandBase {

    private final ClimbSubsystem subsystem;

    public NeutralClimbHookCommand(ClimbSubsystem subsystem) {
        this.subsystem = subsystem;
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
