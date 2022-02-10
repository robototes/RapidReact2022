package frc.team2412.robot.commands.diagnostic;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.commands.intake.IntakeExtendCommand;
import frc.team2412.robot.commands.intake.IntakeInCommand;
import frc.team2412.robot.commands.intake.IntakeOutCommand;
import frc.team2412.robot.commands.intake.IntakeRetractCommand;
import frc.team2412.robot.commands.intake.IntakeStopCommand;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class DiagnosticIntakeCommandGroup extends SequentialCommandGroup {
    public DiagnosticIntakeCommandGroup(IntakeSubsystem intakeSubsystem) {
        addCommands(
                new IntakeInCommand(intakeSubsystem), new WaitCommand(5),
                new IntakeOutCommand(intakeSubsystem), new WaitCommand(5),
                new IntakeStopCommand(intakeSubsystem), new WaitCommand(5),
                new IntakeExtendCommand(intakeSubsystem), new WaitCommand(5),
                new IntakeRetractCommand(intakeSubsystem), new WaitCommand(5));
    }
}
