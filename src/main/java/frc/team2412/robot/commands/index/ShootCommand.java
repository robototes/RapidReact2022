package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.team2412.robot.commands.shooter.ShooterTargetCommand;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.ShooterVisionSubsystem;

/**
 * Subsystems: {@link IndexSubsystem}, {@link ShooterSubsystem}, {@link ShooterVisionSubsystem}
 */
public class ShootCommand extends ParallelCommandGroup {

    public ShootCommand() {
        addCommands(new IndexShootCommand(), new ShooterTargetCommand());
    }
}
