package frc.team2412.robot.commands.diagnostic;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.commands.intake.IntakeSetExtendCommand;
import frc.team2412.robot.commands.intake.IntakeSetInCommand;
import frc.team2412.robot.commands.intake.IntakeSetOutCommand;
import frc.team2412.robot.commands.intake.IntakeSetRetractCommand;
import frc.team2412.robot.commands.intake.IntakeSetStopCommand;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class DiagnosticIntakeCommandGroup extends SequentialCommandGroup {
    public DiagnosticIntakeCommandGroup(IntakeSubsystem intakeSubsystem) {
        super(
                new IntakeSetExtendCommand(intakeSubsystem),
                new WaitCommand(0.5),
                new IntakeSetInCommand(intakeSubsystem),
                new WaitCommand(0.5),
                new IntakeSetOutCommand(intakeSubsystem),
                new WaitCommand(0.5),
                new IntakeSetStopCommand(intakeSubsystem),
                new WaitCommand(0.5),
                new IntakeSetRetractCommand(intakeSubsystem));
    }
}
