package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.subsystem.IndexSubsystem;

public class IndexTestCommand extends SequentialCommandGroup {
  public IndexTestCommand(IndexSubsystem subsystem) {
    addCommands(new IndexShootCommand(subsystem));
  }
}
