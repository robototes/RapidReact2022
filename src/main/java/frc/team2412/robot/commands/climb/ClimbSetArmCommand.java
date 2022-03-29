package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ClimbSubsystem;

public class ClimbSetArmCommand extends CommandBase {

    private ClimbSubsystem subsystem;
    private double value;

    public ClimbSetArmCommand(ClimbSubsystem subsystem, double value) {
        this.subsystem = subsystem;
        this.value = value;
    }

    @Override
    public void execute() {
        subsystem.setMotorSpeed(value);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopArm(true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
