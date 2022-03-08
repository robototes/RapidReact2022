package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeStopCommand extends CommandBase {

    private final IntakeSubsystem subsystem;

    public IntakeStopCommand(IntakeSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.intakeStop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
