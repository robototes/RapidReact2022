package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IndexSubsystem;

public class IndexSecondMotorOutCommand extends CommandBase {
    private final IndexSubsystem subsystem;

    public IndexSecondMotorOutCommand(IndexSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.secondMotorOut();
    }

    @Override
    public void execute() {
        if (!subsystem.secondSensorHasBallIn()) {
            subsystem.secondMotorStop();
        }
    }

    @Override
    public boolean isFinished() {
        return !subsystem.isSecondMotorOn();
    }
}