package frc.team2412.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.WpilibDrivebaseSubsystem;

public class WPILIBdriveCommand extends CommandBase {

    WpilibDrivebaseSubsystem drive;
    double X, y, rotation;

    public WPILIBdriveCommand(WpilibDrivebaseSubsystem drive, double X, double y, double rotation) {
        addRequirements(drive);
        this.drive = drive;
        this.X = X;
        this.y = y;
        this.rotation = rotation;
    }

    @Override
    public void execute() {
        drive.drive(X, y, rotation, false);
    }

}
