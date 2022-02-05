package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeOutCommand extends CommandBase {

    public final IntakeSubsystem subsystem;

    public IntakeOutCommand(IntakeSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);

    }

    @Override
    public void execute() {
        subsystem.intakeOut();

    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
