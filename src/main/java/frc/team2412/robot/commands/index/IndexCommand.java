package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IndexSubsystem;

public class IndexCommand extends CommandBase {
    private IndexSubsystem subsystem;

    public IndexCommand(IndexSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if (!subsystem.feederSensorHasBallIn()) {
            subsystem.ingestMotorIn();
            subsystem.feederMotorIn();
        } else {
            subsystem.feederMotorStop();
            subsystem.ingestMotorStop();
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
