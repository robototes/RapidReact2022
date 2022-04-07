package frc.team2412.robot.commands.intake;

import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.INTAKE_IN_SPEED;

import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeSetInCommand extends IntakeSetCommand {
  public IntakeSetInCommand(IntakeSubsystem subsystem) {
    super(subsystem, INTAKE_IN_SPEED, true);
  }
}
