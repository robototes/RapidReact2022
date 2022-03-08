package frc.team2412.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class ShooterHoodMotorStopCommand extends InstantCommand {
    /**
     * Subsystems: {@link ShooterSubsystem}
     */
    public ShooterHoodMotorStopCommand(ShooterSubsystem shooter) {
        super(shooter::stopHoodMotor, shooter);
    }
}
