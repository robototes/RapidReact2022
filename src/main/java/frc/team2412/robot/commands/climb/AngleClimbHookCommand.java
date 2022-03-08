package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team2412.robot.subsystem.ClimbSubsystem;

/**
 * Subsystems: {@link ClimbSubsystem}
 */
public class AngleClimbHookCommand extends CommandBase {

    private final ClimbSubsystem subsystem;

    public AngleClimbHookCommand() {
        this.subsystem = ClimbSubsystem.instance;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.angleClimbHook(DoubleSolenoid.Value.kForward);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
