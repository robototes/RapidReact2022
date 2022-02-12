package frc.team2412.robot.commands.diagnostic;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.commands.shooter.ShooterHardwareTestCommand;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class DiagnosticShooterCommandGroup extends SequentialCommandGroup {
    public DiagnosticShooterCommandGroup(ShooterSubsystem shooterSubsystem) {
        super(
                new ShooterHardwareTestCommand(shooterSubsystem), new WaitCommand(5));
    }
}
