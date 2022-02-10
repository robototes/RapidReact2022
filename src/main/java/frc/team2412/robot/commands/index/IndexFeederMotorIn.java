package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IndexSubsystem;

public class IndexFeederMotorIn extends CommandBase {
    private final IndexSubsystem subsystem;

    public IndexFeederMotorIn(IndexSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.feederMotorIn();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
