package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private CANSparkMax leftClimberMotor;
    private CANSparkMax rightClimberMotor;
    private DigitalInput swBottomLeftLimitSwitch;
    private DigitalInput swBottomRightLimitSwitch;
    private DigitalInput swTopLeftLimitSwitch;
    private DigitalInput swTopRightLimitSwitch;

    public Climber() {
        leftClimberMotor = new CANSparkMax(11, MotorType.kBrushless);
        rightClimberMotor = new CANSparkMax(12, MotorType.kBrushless);
        swBottomLeftLimitSwitch = new DigitalInput(0);
        swBottomRightLimitSwitch = new DigitalInput(1);
        swTopLeftLimitSwitch = new DigitalInput(2);
        swTopRightLimitSwitch = new DigitalInput(3);
    }

    public void defaultControl(double leftClimberSpeed, double rightClimberSpeed) {
        if (swBottomLeftLimitSwitch.get()) {
            if(leftClimberSpeed > 0) {
                leftClimberSpeed = 0;                
            }
        }
        if (swTopLeftLimitSwitch.get()) {
            if(leftClimberSpeed < 0) {
                leftClimberSpeed = 0;
            }
        }
         if (swBottomRightLimitSwitch.get()) {
            if(rightClimberSpeed > 0) {
                rightClimberSpeed = 0;                
            }
        }
        if (swTopRightLimitSwitch.get()) {
            if(rightClimberSpeed < 0) {
                rightClimberSpeed = 0;
            }
        }
        leftClimberMotor.set(-leftClimberSpeed);
        rightClimberMotor.set(rightClimberSpeed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Bottom Left Limit Switch", swBottomLeftLimitSwitch.get());
        SmartDashboard.putBoolean("Bottom Right Limit Switch", swBottomRightLimitSwitch.get());
        SmartDashboard.putBoolean("Top Left Limit Switch", swTopLeftLimitSwitch.get());
        SmartDashboard.putBoolean("Top Right Limit Switch", swTopRightLimitSwitch.get());
    }
}
