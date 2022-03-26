package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ClimbSubsystem;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class ClimbExtendSlowlyCommand extends CommandBase {

    private ClimbSubsystem climbSubsystem;

    // Needs all subsystems so doesn't run default commands in queue
    public ClimbExtendSlowlyCommand(ClimbSubsystem climbSubsystem, IntakeSubsystem intakeSubsystem,
            IndexSubsystem indexSubsystem, ShooterSubsystem shooterSubsystem) {
        this.climbSubsystem = climbSubsystem;
        addRequirements(climbSubsystem, intakeSubsystem, indexSubsystem, shooterSubsystem);
    }

    @Override
    public void execute() {
        climbSubsystem.extendArmSlowly();
    }

}
