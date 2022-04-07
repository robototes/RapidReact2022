package frc.team2412.robot.commands.intake;

import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeSetExtendCommand extends IntakeSetCommand {

  public IntakeSetExtendCommand(IntakeSubsystem subsystem) {
    super(subsystem, 0, true);
  }
}
