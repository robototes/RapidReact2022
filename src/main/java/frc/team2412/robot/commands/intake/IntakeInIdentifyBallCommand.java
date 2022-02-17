package frc.team2412.robot.commands.intake;

import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeInIdentifyBallCommand extends IntakeMotorInCommand {

    public IntakeInIdentifyBallCommand(IntakeSubsystem subsystem) {
        super(subsystem);
    }

    @Override
    public boolean isFinished() {
        return true; // subsystem.hasOpposingColorCargo();
    }
}
