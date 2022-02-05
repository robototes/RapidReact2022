package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IndexSubsystem;

public class IndexFeederMotorOutCommand extends CommandBase {
    private final IndexSubsystem subsystem;

    public IndexFeederMotorOutCommand(IndexSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.feederMotorOut();
    }

    @Override
    public void execute() {
        if (!subsystem.feederSensorHasBallIn()) {
            subsystem.feederMotorStop();
        }
    }

    @Override
    public boolean isFinished() {
        return !subsystem.isFeederMotorOn();
    }
}
