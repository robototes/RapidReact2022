package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IndexSubsystem;

public class IndexFeederMotorStop extends CommandBase {
    private final IndexSubsystem subsystem;

    public IndexFeederMotorStop(IndexSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.feederMotorStop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
