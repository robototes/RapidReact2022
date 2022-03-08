package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IndexSubsystem;

public class IndexCommand extends CommandBase {
    private IndexSubsystem subsystem;

    public IndexCommand() {
        this.subsystem = IndexSubsystem.instance;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if (!subsystem.feederSensorHasBallIn() || !subsystem.ingestSensorHasBallIn()) {
            subsystem.ingestMotorIn();
        } else {
            subsystem.ingestMotorStop();
        }
    }

    @Override
    public void end(boolean cancel) {
        subsystem.ingestMotorStop();
    }
}
