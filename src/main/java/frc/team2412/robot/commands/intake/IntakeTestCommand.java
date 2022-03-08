package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.commands.diagnostic.DiagnosticIntakeCommandGroup;

public class IntakeTestCommand extends SequentialCommandGroup {

    public IntakeTestCommand() {
        addCommands(
                // first test
                new DiagnosticIntakeCommandGroup(), new WaitCommand(2),

                // test if motors run despite solenoid not being extended
                new IntakeMotorInCommand(), new WaitCommand(0.5),
                new IntakeMotorOutCommand(), new WaitCommand(0.5),
                new IntakeStopCommand(), new WaitCommand(0.5));
    }
}
