package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IndexSubsystem;

/**
 * Subsystems: {@link IndexSubsystem}
 */
public class IndexSpitCommand extends CommandBase {
    private final IndexSubsystem subsystem;

    public IndexSpitCommand() {
        this.subsystem = IndexSubsystem.instance;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // turn on both motors
        subsystem.ingestMotorOut();
        subsystem.feederMotorOut();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.ingestMotorStop();
        subsystem.feederMotorStop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
