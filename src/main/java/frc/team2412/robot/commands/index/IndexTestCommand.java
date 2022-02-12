package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.subsystem.IndexSubsystem;

public class IndexTestCommand extends SequentialCommandGroup {
    private final IndexSubsystem subsystem;

    public IndexTestCommand(IndexSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);

        addCommands(
                new InstantCommand(() -> subsystem.ingestMotorIn()),
                new WaitCommand(1),
                new InstantCommand(() -> subsystem.ingestMotorOut()),
                new WaitCommand(1),
                new InstantCommand(() -> subsystem.ingestMotorStop()),
                new InstantCommand(() -> subsystem.feederMotorIn()),
                new WaitCommand(1),
                new InstantCommand(() -> subsystem.feederMotorOut()),
                new WaitCommand(1),
                new InstantCommand(() -> subsystem.feederMotorStop()));
    }
}
