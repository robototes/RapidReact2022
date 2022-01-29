package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ClimbSubsystem;

/**
 * Climb from the ground to the mid bar by extending then retracting fixed arm
 */
class ClimbGroundMidCommand extends CommandBase {

    private final ClimbSubsystem subsystem;
    private static boolean done = false;

    public ClimbGroundMidCommand(ClimbSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    public void periodic() {
        if (subsystem.getMotorState() == ClimbSubsystem.ClimbConstants.ClimbMotorState.FIXED_STOP) {
            subsystem.retractFixedArm();
            done = true;
        }
    }

    @Override
    public void execute() {
        subsystem.extendFixedArm();
    }

    @Override
    public boolean isFinished() {
        return done;
    }

}