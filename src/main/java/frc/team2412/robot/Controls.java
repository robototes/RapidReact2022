package frc.team2412.robot;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.input.XboxController;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;

import static frc.team2412.robot.Subsystems.SubsystemConstants.*;
import frc.team2412.robot.commands.drive.DriveCommand;

public class Controls {
    public static class ControlConstants {
        public static final int CONTROLLER_PORT = 0;
    }

    public XboxController controller;

    // controls
    public final Button climbFixedArmUp;
    public final Button climbDynamicArmUp;
    // public final Button climbFixedArmDown;
    // public final Button climbDynamicArmDown;
    // public final Button climbRungMovement;

    // drive
    public final Button resetDriveGyro;

    public Subsystems subsystems;

    public Controls(Subsystems s) {
        subsystems = s;
        controller = new XboxController(ControlConstants.CONTROLLER_PORT);

        climbFixedArmUp = controller.getAButton();
        climbDynamicArmUp = controller.getAButton();

        resetDriveGyro = controller.getBackButton();

        if (CLIMB_ENABLED)
            bindClimbControls();
        if (DRIVE_ENABLED)
            bindDriveControls();
        if (INDEX_ENABLED)
            bindIndexControls();
        if (INTAKE_ENABLED)
            bindIntakeControls();
        if (SHOOTER_ENABLED)
            bindShooterControls();
    }

    // TODO these yay

    public void bindClimbControls() {

    }

    public void bindDriveControls() {
        CommandScheduler.getInstance().setDefaultCommand(subsystems.drivebaseSubsystem,
                new DriveCommand(
                        subsystems.drivebaseSubsystem,
                        controller.getLeftYAxis(),
                        controller.getLeftXAxis(),
                        controller.getRightXAxis()));
        resetDriveGyro.whenPressed(() -> {
            subsystems.drivebaseSubsystem.resetGyroAngle(Rotation2.ZERO);
        });

    }

    public void bindIndexControls() {

    }

    public void bindIntakeControls() {

    }

    public void bindShooterControls() {

    }
}
