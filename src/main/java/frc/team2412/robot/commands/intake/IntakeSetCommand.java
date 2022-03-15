package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeSetCommand extends CommandBase {

    public final IntakeSubsystem subsystem;
    private final double speed;
    private final boolean intakeExtend;

    public IntakeSetCommand(IntakeSubsystem subsystem, double speed, boolean intakeExtend) {
        this.subsystem = subsystem;
        this.speed = speed;
        this.intakeExtend = intakeExtend;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if (intakeExtend) {
            subsystem.intakeExtend();
        } else {
            subsystem.intakeRetract();
        }
        subsystem.setSpeed(speed);
    }
}
