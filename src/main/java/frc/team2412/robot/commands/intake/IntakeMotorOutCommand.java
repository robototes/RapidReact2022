package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IntakeSubsystem;

/**
 * Subsystems: {@link IntakeSubsystem}
 */
public class IntakeMotorOutCommand extends CommandBase {

    private final IntakeSubsystem subsystem;

    public IntakeMotorOutCommand() {
        this.subsystem = IntakeSubsystem.instance;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.intakeOut();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.intakeStop();
    }
}
