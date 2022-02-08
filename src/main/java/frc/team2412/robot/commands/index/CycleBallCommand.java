package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.subsystem.IndexSubsystem;

public class CycleBallCommand extends SequentialCommandGroup {
    public CycleBallCommand(IndexSubsystem subsystem) {
        addCommands(
                new IndexFeederMotorInCommand(subsystem),
                new IndexIngestMotorInCommand(subsystem));
        this.addRequirements(subsystem);
    }
}
