package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private TalonSRX frontShooterMotor;

    public Shooter() {
        frontShooterMotor = new TalonSRX(13);
    }

    public void defaultControl(double shooterSpeed) {
        frontShooterMotor.set(ControlMode.PercentOutput, shooterSpeed);
    }

    public Command autoShooterOutCommand() {
        return runOnce(() -> {
            frontShooterMotor.set(ControlMode.PercentOutput, 0.4);
        });
    }

    public Command autoShooterInCommand() {
        return runOnce(() -> {
            frontShooterMotor.set(ControlMode.PercentOutput, -0.4);
        });
    }

    public Command autoShooterOffCommand() {
        return runOnce(() -> {
            frontShooterMotor.set(ControlMode.PercentOutput, 0);
        });
    }

}
