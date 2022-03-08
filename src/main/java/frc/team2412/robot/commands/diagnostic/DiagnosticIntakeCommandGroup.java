package frc.team2412.robot.commands.diagnostic;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.commands.intake.IntakeExtendCommand;
import frc.team2412.robot.commands.intake.IntakeMotorInCommand;
import frc.team2412.robot.commands.intake.IntakeMotorOutCommand;
import frc.team2412.robot.commands.intake.IntakeRetractCommand;
import frc.team2412.robot.commands.intake.IntakeStopCommand;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class DiagnosticIntakeCommandGroup extends SequentialCommandGroup {
    public DiagnosticIntakeCommandGroup(IntakeSubsystem intakeSubsystem) {
        super(
                new IntakeExtendCommand(intakeSubsystem), new WaitCommand(0.5),
                new IntakeMotorInCommand(intakeSubsystem), new WaitCommand(0.5),
                new IntakeMotorOutCommand(intakeSubsystem), new WaitCommand(0.5),
                new IntakeStopCommand(intakeSubsystem), new WaitCommand(0.5),
                new IntakeRetractCommand(intakeSubsystem));
    }
}
