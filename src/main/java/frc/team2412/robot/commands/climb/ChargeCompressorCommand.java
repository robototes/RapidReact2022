package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class ChargeCompressorCommand extends CommandBase {

    public ChargeCompressorCommand(IntakeSubsystem intakeSubsystem,
            IndexSubsystem indexSubsystem, ShooterSubsystem shooterSubsystem, DrivebaseSubsystem drivebaseSubsystem) {
        addRequirements(intakeSubsystem, indexSubsystem, shooterSubsystem, drivebaseSubsystem);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
