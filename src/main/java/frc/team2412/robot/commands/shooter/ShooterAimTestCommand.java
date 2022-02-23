package frc.team2412.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class ShooterAimTestCommand extends InstantCommand {
    public ShooterAimTestCommand(ShooterSubsystem shooter) {
        super(() -> {
            shooter.setFlywheelRPM(shooter.getFlywheelTestRPM());
            shooter.setHoodAngle(shooter.getHoodTestAngle());
        }, shooter);
    }
}
