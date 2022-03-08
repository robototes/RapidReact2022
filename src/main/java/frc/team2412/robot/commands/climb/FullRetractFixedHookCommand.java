package frc.team2412.robot.commands.climb;

import frc.team2412.robot.subsystem.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Subsystems: {@link ClimbSubsystem}
 */
public class FullRetractFixedHookCommand extends CommandBase {

    private final ClimbSubsystem subsystem;

    public FullRetractFixedHookCommand() {
        this.subsystem = ClimbSubsystem.instance;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.retractFixedArmFully();
    }

    // @Override
    // public boolean isFinished() {
    // return subsystem.isFixedFullyExtended();
    // }

    // @Override
    // public void end(boolean interrupted) {
    // subsystem.stopFixedArm();
    // }

}
