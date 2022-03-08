package frc.team2412.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.team2412.robot.commands.intake.IntakeInCommand;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.ShooterVisionSubsystem;

/**
 * Subsystems: {@link IndexSubsystem}, {@link IntakeSubsystem}, {@link ShooterSubsystem},
 * {@link ShooterVisionSubsystem}
 */
public class FullShootCommand extends ParallelCommandGroup {
    public FullShootCommand() {
        addCommands(new ShooterTargetCommand(), new IntakeInCommand());
    }
}
