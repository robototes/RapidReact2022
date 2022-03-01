package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.commands.diagnostic.DiagnosticIntakeCommandGroup;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeTestCommand extends SequentialCommandGroup {

    public IntakeTestCommand(IntakeSubsystem subsystem) {
        addCommands(
                // first test
                new DiagnosticIntakeCommandGroup(subsystem), new WaitCommand(2),

                // test if motors run despite solenoid not being extended
                new IntakeMotorInCommand(subsystem), new WaitCommand(0.5),
                new IntakeMotorOutCommand(subsystem), new WaitCommand(0.5),
                new IntakeStopCommand(subsystem), new WaitCommand(0.5));
    }
}
