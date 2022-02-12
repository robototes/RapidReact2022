package frc.team2412.robot.commands.diagnostic;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.subsystem.ClimbSubsystem;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class DiagnosticRobotCommandGroup extends SequentialCommandGroup {
    public DiagnosticRobotCommandGroup(
            IntakeSubsystem intakeSubsystem,
            ShooterSubsystem shooterSubsystem,
            IndexSubsystem indexSubsystem,
            ClimbSubsystem climbSubsystem) {
        super(
                new DiagnosticIntakeCommandGroup(intakeSubsystem),
                new DiagnosticShooterCommandGroup(shooterSubsystem),
                new DiagnosticIndexCommandGroup(indexSubsystem),
                new DiagnosticClimbCommandGroup(climbSubsystem));
    }
}
