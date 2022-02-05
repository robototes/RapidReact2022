package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeInCommand extends CommandBase {

    public final IntakeSubsystem subsystem;

    public IntakeInCommand(IntakeSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);

    }

    @Override
    public void execute() {
        subsystem.intakeIn();

    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
