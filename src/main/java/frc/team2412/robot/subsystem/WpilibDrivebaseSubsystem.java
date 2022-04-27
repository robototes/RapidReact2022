package frc.team2412.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.util.SwerveModule;

import static frc.team2412.robot.Hardware.*;

public class WpilibDrivebaseSubsystem extends SubsystemBase {

    public SwerveModule frontLeft = new SwerveModule(DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
            DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, DRIVETRAIN_FRONT_LEFT_ENCODER_PORT, DRIVETRAIN_FRONT_LEFT_ENCODER_PORT,
            DRIVETRAIN_INTAKE_CAN_BUS_NAME);

    public WpilibDrivebaseSubsystem() {

    }

}
