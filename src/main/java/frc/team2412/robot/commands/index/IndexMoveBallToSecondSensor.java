package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IndexSubsystem;

public class IndexMoveBallToSecondSensor extends CommandBase {
    private final IndexSubsystem subsystem;

    public IndexMoveBallToSecondSensor(IndexSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.ingestMotorIn();
        subsystem.feederMotorIn();
    }

    @Override
    public boolean isFinished() {
        if (subsystem.feederSensorHasBallIn()) {
            subsystem.ingestMotorStop();
            subsystem.feederMotorStop();
            return true;
        }
        return false;
    }
}
