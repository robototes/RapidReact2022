package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IndexSubsystem;

public class IndexShootCommand extends CommandBase {
    private final IndexSubsystem subsystem;

    public IndexShootCommand(IndexSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // turn on both motors
        subsystem.ingestMotorIn();
        subsystem.feederMotorIn();
    }

    @Override
    public boolean isFinished() {
        // stop motors when there are no balls left
        if (!subsystem.ingestSensorHasBallIn() && !subsystem.feederSensorHasBallIn()) {
            subsystem.ingestMotorStop();
            subsystem.feederMotorStop();
            return true;
        }
        return false;
    }
}