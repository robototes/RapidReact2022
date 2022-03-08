package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.team2412.robot.commands.shooter.ShooterMisfireCommand;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.ShooterVisionSubsystem;

public class ShooterMissCommand extends ParallelCommandGroup {
    /**
     * Subsystems: {@link IndexSubsystem}, {@link ShooterSubsystem}, {@link ShooterVisionSubsystem}
     */
    public ShooterMissCommand() {
        addCommands(new IndexShootCommand(), new ShooterMisfireCommand());
    }
}
