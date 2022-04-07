package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team2412.robot.commands.index.IndexSpitCommand;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class SpitBallCommand extends ParallelCommandGroup {
  public SpitBallCommand(IndexSubsystem indexSubsystem, IntakeSubsystem intakeSubsystem) {
    addCommands(new IntakeSetOutCommand(intakeSubsystem), new IndexSpitCommand(indexSubsystem));
  }
}
