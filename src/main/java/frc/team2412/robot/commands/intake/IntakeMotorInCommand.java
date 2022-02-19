package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeMotorInCommand extends CommandBase {

    protected final IntakeSubsystem subsystem;

    public IntakeMotorInCommand(IntakeSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.intakeIn();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
