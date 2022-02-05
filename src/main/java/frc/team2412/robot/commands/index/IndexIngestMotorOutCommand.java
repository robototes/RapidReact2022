package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IndexSubsystem;

public class IndexIngestMotorOutCommand extends CommandBase {
    private final IndexSubsystem subsystem;

    public IndexIngestMotorOutCommand(IndexSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.ingestMotorOut();
    }

    @Override
    public void execute() {
        if (!subsystem.ingestSensorHasBallIn()) {
            subsystem.ingestMotorStop();
        }
    }

    @Override
    public boolean isFinished() {
        return !subsystem.isIngestMotorOn();
    }
}
