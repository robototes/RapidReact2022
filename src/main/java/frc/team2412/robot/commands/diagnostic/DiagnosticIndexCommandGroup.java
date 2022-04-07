package frc.team2412.robot.commands.diagnostic;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.commands.index.IndexTestCommand;
import frc.team2412.robot.subsystem.IndexSubsystem;

public class DiagnosticIndexCommandGroup extends SequentialCommandGroup {
  public DiagnosticIndexCommandGroup(IndexSubsystem indexSubsystem) {
    super(new IndexTestCommand(indexSubsystem));
  }
}
