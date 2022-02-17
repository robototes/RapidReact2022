package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IndexSubsystem;

public class IndexMoveBallToFirstSensor extends CommandBase {
    private final IndexSubsystem subsystem;

    public IndexMoveBallToFirstSensor(IndexSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.ingestMotorIn();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.ingestMotorStop();
    }

    @Override
    public boolean isFinished() {
        return subsystem.ingestSensorHasBallIn();
    }
}
