package frc.team2412.robot;

import static frc.team2412.robot.Subsystems.SubsystemConstants.*;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.input.XboxController;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;
import static frc.team2412.robot.Subsystems.SubsystemConstants.*;
import static frc.team2412.robot.Controls.ControlConstants.*;
import frc.team2412.robot.commands.drive.DriveCommand;
import frc.team2412.robot.commands.intake.IntakeExtendCommand;
import frc.team2412.robot.commands.intake.IntakeRetractCommand;

public class Controls {
    public static class ControlConstants {
        public static final int CONTROLLER_PORT = 0;

        public static final XboxController controller = new XboxController(ControlConstants.CONTROLLER_PORT);

        // Climb
        public static final Button climbFixedArmUp = controller.getAButton();
        public static final Button climbDynamicArmUp = controller.getAButton();
        // public final Button
        // climbFixedArmDown;
        // public final Button climbDynamicArmDown;
        // public final Button climbRungMovement;

        // Drive
        public static final Button resetDriveGyro = controller.getBackButton();

        // Intake
        public static final Button buttonIntakeExtend = controller.getLeftBumperButton();
        public static final Button buttonIntakeRetract = controller.getRightBumperButton();

    }

    public Subsystems subsystems;

    public Controls(Subsystems s) {
        subsystems = s;

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
                        controller.getRightXAxis(),
                        true)); // this parameter controls if robot drives field oriented
        resetDriveGyro.whenPressed(() -> {
            subsystems.drivebaseSubsystem.resetGyroAngle(Rotation2.ZERO);
        });

    }

    public void bindIndexControls() {

    }

    public void bindIntakeControls() {
        if (INTAKE_ENABLED) {
            buttonIntakeExtend.whenPressed(new IntakeExtendCommand(subsystems.intakeSubsystem));
            buttonIntakeRetract.whenPressed(new IntakeRetractCommand(subsystems.intakeSubsystem));
        }
    }

    public void bindShooterControls() {

    }
}
