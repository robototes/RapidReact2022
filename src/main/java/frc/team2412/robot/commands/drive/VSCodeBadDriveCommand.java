package frc.team2412.robot.commands.drive;

import org.frcteam2910.common.robot.input.XboxController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.WpilibDrivebaseSubsystem;

public class VSCodeBadDriveCommand extends CommandBase {

    private WpilibDrivebaseSubsystem drive;
    private double x, y, rotation;
    private XboxController driveController;

    public VSCodeBadDriveCommand(WpilibDrivebaseSubsystem drive, XboxController driveController) {
        addRequirements(drive);
        this.drive = drive;
        this.driveController = driveController;
    }

    @Override
    public void execute() {
        x = driveController.getLeftXAxis().get();
        y = driveController.getLeftYAxis().get();
        rotation = driveController.getRightXAxis().get();

        drive.drive(x * drive.maxVelocityMetersPerSecond, y * drive.maxVelocityMetersPerSecond,
                rotation * drive.maxVelocityMetersPerSecond);
    }

}
