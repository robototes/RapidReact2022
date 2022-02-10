package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IndexSubsystem;

public class IndexIngestMotorIn extends CommandBase {
    private final IndexSubsystem subsystem;

    public IndexIngestMotorIn(IndexSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.ingestMotorIn();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
