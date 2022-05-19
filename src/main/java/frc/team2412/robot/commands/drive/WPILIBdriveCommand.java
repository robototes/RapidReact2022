package frc.team2412.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.WpilibDrivebaseSubsystem;

public class WPILibDriveCommand extends CommandBase {

    private WpilibDrivebaseSubsystem drive;
    private double x, y, rotation;

    public WPILibDriveCommand(WpilibDrivebaseSubsystem drive, double x, double y, double rotation) {
        addRequirements(drive);
        this.drive = drive;
        this.x = x;
        this.y = y;
        this.rotation = rotation;
    }

    @Override
    public void execute() {
        drive.drive(x * drive.maxVelocityMetersPerSecond, y * drive.maxVelocityMetersPerSecond,
                rotation * drive.maxVelocityMetersPerSecond);
    }

}
