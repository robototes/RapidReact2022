package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.subsystem.IndexSubsystem;

public class IndexTestCommand extends SequentialCommandGroup {
    private final IndexSubsystem subsystem;

    public IndexTestCommand(IndexSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);

        addCommands(
                new IndexIngestMotorIn(subsystem),
                new WaitCommand(1),
                new IndexIngestMotorOut(subsystem),
                new WaitCommand(1),
                new IndexIngestMotorStop(subsystem),
                new IndexFeederMotorIn(subsystem),
                new WaitCommand(1),
                new IndexFeederMotorOut(subsystem),
                new WaitCommand(1),
                new IndexFeederMotorStop(subsystem));
    }
}
