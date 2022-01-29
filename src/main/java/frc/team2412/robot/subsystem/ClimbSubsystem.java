package frc.team2412.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

    public static class ClimbConstants {
        public static final double MAX_SPEED = 1;
        public static final double STOP_SPEED = 0;
    }

    /**
     * State of the cliimb subsystem
     */
    enum ClimbSubsystemState {
        ENABLED, DISABLED
    }

    /**
     * State of the solenoid to change the climb angle
     */
    enum HookArmState {
        EXTENDED, UPRIGHT
    }

    private final TalonFX climbMotor1;
    private final TalonFX climbMotor2;

    private final DoubleSolenoid solenoid;

    ClimbSubsystemState state;

    public ClimbSubsystem(TalonFX climbFixed1, TalonFX climbFixed2, DoubleSolenoid climbAngle) {
        climbMotor1 = climbFixed1;
        climbMotor2 = climbFixed2;
        solenoid = climbAngle;
        setName("ClimbSubsystem");
    }

    @Override
    public void periodic() {
        System.out.println("Subsystem " + getName() + " says hi?");
    }

    public void setEnabled() {
        state = ClimbSubsystemState.ENABLED;
    }

     public void setDisabled() {
        state = ClimbSubsystemState.DISABLED;
    }

    /**
     * Extend the solenoid to angle the climb hook
     * @param extended - True to extend the arm, false to collapse
     */
    public void angleClimbHook(boolean extended) {
        solenoid.set(extended ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }  
    
    public void extendFixedArm() {
        climbMotor1.set(TalonFXControlMode.PercentOutput, ClimbConstants.MAX_SPEED);
    }
    
    public void retractFixedArm() {
        climbMotor1.set(TalonFXControlMode.PercentOutput, -ClimbConstants.MAX_SPEED);
    }

    public void stopFixedArm() {
        climbMotor1.set(TalonFXControlMode.PercentOutput, ClimbConstants.STOP_SPEED);
    }

    public void extendAngledArm() {
        climbMotor2.set(TalonFXControlMode.PercentOutput, ClimbConstants.MAX_SPEED);
    }
    
    public void retractAngledArm() {
        climbMotor2.set(TalonFXControlMode.PercentOutput, -ClimbConstants.MAX_SPEED);
    }

    public void stopAngledArm() {
        climbMotor2.set(TalonFXControlMode.PercentOutput, ClimbConstants.STOP_SPEED);
    }
    
}