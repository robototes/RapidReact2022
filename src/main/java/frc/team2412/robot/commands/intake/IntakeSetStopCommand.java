package frc.team2412.robot.commands.intake;

import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeSetStopCommand extends IntakeSetCommand {

  public IntakeSetStopCommand(IntakeSubsystem subsystem) {
    super(subsystem, 0, false);
  }

  @Override
  public void execute() {
    super.subsystem.intakeStop();
  }
}
