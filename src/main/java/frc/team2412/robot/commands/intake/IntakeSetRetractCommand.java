package frc.team2412.robot.commands.intake;

import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeSetRetractCommand extends IntakeSetCommand {

  public IntakeSetRetractCommand(IntakeSubsystem subsystem) {
    super(subsystem, 0, false);
  }
}
