package frc.team2412.robot;

import static frc.team2412.robot.Subsystems.SubsystemConstants.CLIMB_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.DRIVE_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.INDEX_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.INTAKE_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.SHOOTER_ENABLED;

import org.frcteam2910.common.robot.input.DPadButton.Direction;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.input.XboxController;

import edu.wpi.first.wpilibj2.command.button.Button;
import frc.team2412.robot.commands.intake.IntakeExtendCommand;
import frc.team2412.robot.commands.intake.IntakeInCommand;
import frc.team2412.robot.commands.intake.IntakeOutCommand;
import frc.team2412.robot.commands.intake.IntakeRetractCommand;

public class Controls {
    public static class ControlConstants {
        public static final int CONTROLLER_PORT = 0;
        public static final int CODRIVER_CONTROLLER_PORT = 1;
    }

    public XboxController driveController;

    public XboxController codriverController;

    // controls
    // public final Button climbFixedArmUp;
    // public final Button climbDynamicArmUp;
    // public final Button climbFixedArmDown;
    // public final Button climbDynamicArmDown;
    // public final Button climbRungMovement;

    // shooter
    public final Button shootButton = driveController.getLeftBumperButton();
    public final Button hoodUpButton = driveController.getDPadButton(Direction.UP);
    public final Button hoodDownButton = driveController.getDPadButton(Direction.DOWN);
    public final Button turretLeftButton = driveController.getDPadButton(Direction.LEFT);
    public final Button turretRightButton = driveController.getDPadButton(Direction.RIGHT);

    // intake
    public final Button intakeInButton = driveController.getRightBumperButton();
    public final Button intakeSpitButton = driveController.getAButton();
    public final Button intakeRetractButton = driveController.getBButton();

    // drive
    public final Button resetDriveGyroButton = driveController.getRightJoystickButton();

    // climb
    public final Button fixedArmUpManualButton = codriverController.getDPadButton(Direction.UP);
    public final Button fixedArmDownManualButton = codriverController.getDPadButton(Direction.DOWN);
    public final Button dynamicArmUpManualButton = codriverController.getDPadButton(Direction.LEFT);
    public final Button dynamicArmDownManualButton = codriverController.getDPadButton(Direction.RIGHT);

    public final Button fixedArmUpButton = codriverController.getXButton();
    public final Button fixedArmDownButton = codriverController.getYButton();
    public final Button dynamicArmUpButton = codriverController.getAButton();
    public final Button dynamicArmDownButton = codriverController.getBButton();

    public final Button rungClimbButton = codriverController.getRightBumperButton();

    public Subsystems subsystems;

    public Controls(Subsystems s) {
        subsystems = s;
        driveController = new XboxController(ControlConstants.CONTROLLER_PORT);
        codriverController = new XboxController(ControlConstants.CODRIVER_CONTROLLER_PORT);

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
        if (DRIVE_ENABLED) {
            resetDriveGyroButton.whenPressed(() -> {

                subsystems.drivebaseSubsystem.resetGyroAngle(Rotation2.ZERO);
            });
        }
    }

    public void bindIndexControls() {

    }

    public void bindIntakeControls() {
        if (INTAKE_ENABLED) {
            intakeInButton.whenPressed(new IntakeExtendCommand(subsystems.intakeSubsystem)
                    .andThen(new IntakeInCommand(subsystems.intakeSubsystem)));
            intakeRetractButton.whenPressed(new IntakeRetractCommand(subsystems.intakeSubsystem));
            intakeSpitButton.whenPressed(new IntakeOutCommand(subsystems.intakeSubsystem));
        }
    }

    public void bindShooterControls() {
        shootButton.whenPressed(command);
    }
}
