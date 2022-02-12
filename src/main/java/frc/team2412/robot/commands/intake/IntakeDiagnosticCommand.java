package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeDiagnosticCommand extends SequentialCommandGroup {

    private final IntakeSubsystem subsystem;

    public IntakeDiagnosticCommand(IntakeSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
        addCommands(new IntakeExtendCommand(subsystem), new IntakeInCommand(subsystem),
                new IntakeOutCommand(subsystem), new IntakeStopCommand(subsystem), new IntakeRetractCommand(subsystem));
    }
}
