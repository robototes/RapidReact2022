package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IndexSubsystem;

/**
 * Subsystems: {@link IndexSubsystem}
 */
public class IndexShootCommand extends CommandBase {
    private final IndexSubsystem subsystem;

    public IndexShootCommand() {
        this.subsystem = IndexSubsystem.instance;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // turn on both motors
        subsystem.ingestMotorIn();
        subsystem.feederMotorIn();
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
