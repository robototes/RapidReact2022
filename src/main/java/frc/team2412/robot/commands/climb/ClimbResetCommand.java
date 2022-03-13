package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ClimbSubsystem;

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
    }

    @Override
    public boolean isFinished() {
        return climbSubsystem.isHittingLimitSwitch();
    }

}
