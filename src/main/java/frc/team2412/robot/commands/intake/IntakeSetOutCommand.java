package frc.team2412.robot.commands.intake;

import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.INTAKE_OUT_SPEED;

import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeSetOutCommand extends IntakeSetCommand {
    public IntakeSetOutCommand(IntakeSubsystem subsystem) {
        super(subsystem, INTAKE_OUT_SPEED, true);
    }
}
