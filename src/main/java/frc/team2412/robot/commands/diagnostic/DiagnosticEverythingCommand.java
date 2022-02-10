package frc.team2412.robot.commands.diagnostic;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class DiagnosticEverythingCommand extends SequentialCommandGroup {
    public DiagnosticEverythingCommand(IntakeSubsystem intakeSubsystem) {
        addCommands(new DiagnosticIntakeCommandGroup(intakeSubsystem));
    }
}
