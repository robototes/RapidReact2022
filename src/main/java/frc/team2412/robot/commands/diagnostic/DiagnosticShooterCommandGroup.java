package frc.team2412.robot.commands.diagnostic;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.team2412.robot.commands.shooter.ShooterDiagnosticCommand;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class DiagnosticShooterCommandGroup extends SequentialCommandGroup {
    /**
     * Subsystems: {@link ShooterSubsystem}
     */
    public DiagnosticShooterCommandGroup() {
        super(new ShooterDiagnosticCommand());
    }
}
