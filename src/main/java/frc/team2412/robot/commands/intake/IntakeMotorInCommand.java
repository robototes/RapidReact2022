package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeMotorInCommand extends CommandBase {
    private final IntakeSubsystem subsystem;

    /**
     * Subsystems: {@link IntakeSubsystem}
     */
    public IntakeMotorInCommand() {
        this.subsystem = IntakeSubsystem.instance;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.intakeIn();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
