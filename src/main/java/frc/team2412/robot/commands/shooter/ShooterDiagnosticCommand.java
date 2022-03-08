package frc.team2412.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.team2412.robot.subsystem.ShooterSubsystem.ShooterConstants;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class ShooterDiagnosticCommand extends SequentialCommandGroup {
    /**
     * Subsystems: {@link ShooterSubsystem}
     */
    public ShooterDiagnosticCommand() {
        addCommands(new ShooterTurretSetAngleCommand(ShooterConstants.MIN_TURRET_ANGLE),
                new ShooterTurretSetAngleCommand(ShooterConstants.MAX_TURRET_ANGLE),
                new ShooterTurretSetAngleCommand(
                        (ShooterConstants.MIN_TURRET_ANGLE + ShooterConstants.MAX_TURRET_ANGLE) / 2),
                new ShooterHoodSetConstantAngleCommand(ShooterConstants.MAX_HOOD_ANGLE),
                new ShooterHoodSetConstantAngleCommand(ShooterConstants.MIN_HOOD_ANGLE),
                new ShooterFlywheelStartCommand(),
                new WaitCommand(1),
                new ShooterFlywheelStopCommand());
    }
}
