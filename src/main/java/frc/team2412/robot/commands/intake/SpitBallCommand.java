package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.team2412.robot.commands.index.IndexSpitCommand;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;

/**
 * Subsystems: {@link IndexSubsystem}, {@link IntakeSubsystem}
 */
public class SpitBallCommand extends ParallelCommandGroup {
    public SpitBallCommand() {
        addCommands(new IntakeMotorOutCommand(), new IndexSpitCommand());
    }
}
