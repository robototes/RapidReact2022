package frc.team2412.robot;

import static frc.team2412.robot.Subsystems.SubsystemConstants.*;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.input.XboxController;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;

import frc.team2412.robot.commands.climb.ExtendAngledHookCommand;
import frc.team2412.robot.commands.climb.ExtendFixedHookCommand;
import frc.team2412.robot.commands.climb.RetractAngledHookCommand;
import frc.team2412.robot.commands.climb.RetractFixedHookCommand;
import frc.team2412.robot.commands.climb.ClimbStageChooserCommand;
import frc.team2412.robot.commands.drive.DriveCommand;
import frc.team2412.robot.commands.intake.IntakeExtendCommand;
import frc.team2412.robot.commands.intake.IntakeRetractCommand;

public class Controls {
    public static class ControlConstants {
        public static final int CONTROLLER_PORT = 0;
    }

    public XboxController controller;

    // climb
    public final Button climbFixedArmUp;
    public final Button climbDynamicArmUp;
    public final Button climbFixedArmDown;
    public final Button climbDynamicArmDown;
    public final Button climbRungMovement;

    // intake
    public final Button buttonIntakeRetract;
    public final Button buttonIntakeExtend;

    // drive
    public final Button resetDriveGyro;

    public Subsystems subsystems;

    public Controls(Subsystems s) {
        subsystems = s;
        controller = new XboxController(ControlConstants.CONTROLLER_PORT);

        climbFixedArmUp = controller.getYButton();
        climbDynamicArmUp = controller.getBButton();
        climbFixedArmDown = controller.getAButton();
        climbDynamicArmDown = controller.getXButton();
        climbRungMovement = controller.getRightBumperButton();

        buttonIntakeExtend = controller.getLeftBumperButton();
        buttonIntakeRetract = controller.getRightBumperButton();

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
        climbFixedArmUp.whenPressed(new ExtendFixedHookCommand(subsystems.climbSubsystem));
        climbFixedArmDown.whenPressed(new RetractFixedHookCommand(subsystems.climbSubsystem));
        climbDynamicArmUp.whenPressed(new ExtendAngledHookCommand(subsystems.climbSubsystem));
        climbDynamicArmDown.whenPressed(new RetractAngledHookCommand(subsystems.climbSubsystem));
        climbRungMovement.whenPressed(new ClimbStageChooserCommand(subsystems.climbSubsystem));
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
