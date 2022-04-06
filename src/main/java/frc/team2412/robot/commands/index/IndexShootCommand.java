package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.TargetLocalizer;

public class IndexShootCommand extends CommandBase {
    private final IndexSubsystem subsystem;
    private final TargetLocalizer localizer;

    public IndexShootCommand(IndexSubsystem subsystem, TargetLocalizer targetLocalizer) {
        this.subsystem = subsystem;
        localizer = targetLocalizer;
        addRequirements(subsystem);
    }

    public IndexShootCommand(IndexSubsystem subsystem) {
        this(subsystem, null);
    }

    @Override
    public void execute() {
        // turn on both motors
        if (localizer == null || localizer.upToSpeed()) {
            subsystem.ingestMotorShoot();
            subsystem.feederMotorShoot();
        } else {
            subsystem.ingestMotorStop();
            subsystem.feederMotorStop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.ingestMotorStop();
        subsystem.feederMotorStop();
        System.out.println("++++++++++++++++SHOOTING DONE+++++++++++++++++++");

    }
}
