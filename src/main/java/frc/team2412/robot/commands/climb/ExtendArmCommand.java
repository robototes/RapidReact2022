package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ClimbSubsystem;

public class ExtendArmCommand extends CommandBase {

  private final ClimbSubsystem subsystem;

  public ExtendArmCommand(ClimbSubsystem subsystem) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    subsystem.extendArm();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
