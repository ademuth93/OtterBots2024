package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private CANSparkMax leftClimberMotor;
    private CANSparkMax rightClimberMotor;
    private DigitalInput swLeftLimitSwitch;
    private DigitalInput swRightLimitSwitch;

    public Climber() {
        leftClimberMotor = new CANSparkMax(11, MotorType.kBrushless);
        rightClimberMotor = new CANSparkMax(12, MotorType.kBrushless);
        swLeftLimitSwitch = new DigitalInput(0);
        swRightLimitSwitch = new DigitalInput(1);
    }

    public void defaultControl(double leftClimberSpeed, double rightClimberSpeed) {
        if (swLeftLimitSwitch.get()) {
            if (leftClimberSpeed > 0) {
                leftClimberMotor.set(0.02);
            } else {
                leftClimberMotor.set(-leftClimberSpeed);
            }
        } else {
            leftClimberMotor.set(-leftClimberSpeed);
        }

        if (swRightLimitSwitch.get()) {
            if (rightClimberSpeed > 0) {
                rightClimberMotor.set(0.02);
            } else {
                rightClimberMotor.set(rightClimberSpeed);
            }
        } else {
            rightClimberMotor.set(rightClimberSpeed);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Left Limit Switch", swLeftLimitSwitch.get());
        SmartDashboard.putBoolean("Right Limit Switch", swRightLimitSwitch.get());
    }
}
