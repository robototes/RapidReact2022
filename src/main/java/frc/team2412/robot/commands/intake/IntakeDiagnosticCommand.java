package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeDiagnosticCommand extends SequentialCommandGroup {

    private final IntakeSubsystem subsystem;

    public IntakeDiagnosticCommand(IntakeSubsystem subsystem) {
        this.subsystem = subsystem;
        addCommands(new IntakeExtendCommand(subsystem), new WaitCommand(0.5),
                    new IntakeInCommand(subsystem), new WaitCommand(0.5),
                    new IntakeOutCommand(subsystem), new WaitCommand(0.5),
                    new IntakeStopCommand(subsystem), new WaitCommand(0.5),
                    new IntakeRetractCommand(subsystem));
    }
}
