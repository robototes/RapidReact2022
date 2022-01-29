package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ClimbSubsystem;
import frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.ClimbMotorState;

/**
 * Climb from one bar to another
 * 
 * 1. Extend the solenoid, angling the dynamic arm
 * 2. Extend the dynamic arm while also letting go of the fixed arm?
 * 3. Retract the angled arm to move to the next bar
 * might not work?
 */
public class ClimbInterBarCommand extends CommandBase {
    
    private final ClimbSubsystem subsystem;
    private static boolean done = false;
    private static boolean extendFixed = false;

    public ClimbInterBarCommand(ClimbSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    public void periodic() {
        if (!extendFixed && subsystem.getMotorState() == ClimbMotorState.ANGLED_STOP) {
            subsystem.extendFixedArm();
            subsystem.retractAngledArm();
            extendFixed = true;
        }
    }

    @Override
    public void execute() {
        subsystem.angleClimbHook(true);
        subsystem.extendAngledArm();
    }

    @Override
    public boolean isFinished() {
        return done;
    }

}
