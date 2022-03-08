package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeRetractCommand extends CommandBase {
    private final IntakeSubsystem subsystem;

    /**
     * Subsystems: {@link IntakeSubsystem}
     */
    public IntakeRetractCommand() {
        this.subsystem = IntakeSubsystem.instance;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.intakeRetract();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
