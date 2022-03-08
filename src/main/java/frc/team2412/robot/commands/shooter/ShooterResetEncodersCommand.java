package frc.team2412.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class ShooterResetEncodersCommand extends InstantCommand {
    public ShooterResetEncodersCommand() {
        super(() -> {
            ShooterSubsystem.instance.resetHoodEncoder(true);
            ShooterSubsystem.instance.resetTurretEncoder(true);
        }, ShooterSubsystem.instance);
    }
}
