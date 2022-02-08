package frc.team2412.robot;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.input.XboxController;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.GenericHID.*;

import org.frcteam2910.common.robot.input.DPadButton.Direction;

import static frc.team2412.robot.Subsystems.SubsystemConstants.*;
import frc.team2412.robot.commands.climb.*;
import frc.team2412.robot.commands.drive.DriveCommand;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;

public class Controls {
    public static class ControlConstants {
        public static final int CONTROLLER_PORT = 0;
    }

    public XboxController controller;

    // controls

    // climb
    public final Button buttonFixedArmUp;
    public final Button buttonFixedArmDown;
    public final Button buttonDynamicArmUp;
    public final Button buttonDynamicArmDown;
    public final Button buttonAngleDynamicArm;
    public final Button buttonUnangleDynamicArm;
    public final Button buttonNeutralDynamicArm;

    // drive
    public final Button resetDriveGyro;

    public Subsystems subsystems;

    public Controls(Subsystems s) {
        subsystems = s;
        controller = new XboxController(ControlConstants.CONTROLLER_PORT);

        buttonFixedArmUp = controller.getAButton();
        buttonFixedArmDown = controller.getBButton();
        buttonDynamicArmUp = controller.getXButton();
        buttonDynamicArmDown = controller.getYButton();
        buttonAngleDynamicArm = controller.getDPadButton(Direction.RIGHT);
        buttonUnangleDynamicArm = controller.getDPadButton(Direction.LEFT);
        buttonNeutralDynamicArm = controller.getDPadButton(Direction.UP);

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
        buttonFixedArmUp.whenPressed(new ExtendFixedHookCommand(subsystems.climbSubsystem));
        buttonFixedArmDown.whenPressed(new RetractFixedHookCommand(subsystems.climbSubsystem));
        buttonDynamicArmUp.whenPressed(new ExtendAngledHookCommand(subsystems.climbSubsystem));
        buttonDynamicArmDown.whenPressed(new RetractAngledHookCommand(subsystems.climbSubsystem));
        buttonAngleDynamicArm.whenPressed(new AngleClimbHookCommand(subsystems.climbSubsystem));
        buttonUnangleDynamicArm.whenPressed(new UnangleClimbHookCommand(subsystems.climbSubsystem));
    }

    public void bindDriveControls() {
        CommandScheduler.getInstance().setDefaultCommand(subsystems.drivebaseSubsystem,
                new DriveCommand(
                        subsystems.drivebaseSubsystem, 
                        controller.getLeftYAxis(), 
                        controller.getLeftXAxis(),
                        controller.getRightXAxis()
                ));
        resetDriveGyro.whenPressed(() -> {subsystems.drivebaseSubsystem.resetGyroAngle(Rotation2.ZERO);});
        
    }

    public void bindIndexControls() {

    }

    public void bindIntakeControls() {

    }

    public void bindShooterControls() {

    }
}
