package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ClimbSubsystem;

/**
 * Reset the climb subsystem, by lowering the arm fully, 
 * resetting the encoder and stopping the arm once the
 * limit switch has been graciously reached.
 */
public class ClimbResetCommand extends CommandBase {

    private ClimbSubsystem climbSubsystem;

    public ClimbResetCommand(ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;
        addRequirements(climbSubsystem);
    }

    @Override
    public void execute() {
        climbSubsystem.lowerArm();
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.resetEncoder(true);
        climbSubsystem.stopArm(true);
    }

    @Override
    public boolean isFinished() {
        return climbSubsystem.isHittingLimitSwitch();
    }

}
