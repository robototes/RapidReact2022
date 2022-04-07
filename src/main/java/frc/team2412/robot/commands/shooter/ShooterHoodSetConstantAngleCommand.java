package frc.team2412.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class ShooterHoodSetConstantAngleCommand extends CommandBase {
  private final ShooterSubsystem shooter;
  private final double hoodAngle;

  public ShooterHoodSetConstantAngleCommand(ShooterSubsystem shooter, double angle) {
    this.shooter = shooter;
    this.hoodAngle = angle;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    shooter.setHoodAngle(hoodAngle);
  }

  @Override
  public boolean isFinished() {
    return shooter.isHoodAtAngle(hoodAngle);
  }
}
