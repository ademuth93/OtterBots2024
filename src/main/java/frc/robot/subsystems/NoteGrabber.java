package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteGrabber extends SubsystemBase {
    private CANSparkMax frontGrabberMotor;
    private CANSparkMax backGrabberMotor;

    public NoteGrabber() {
        frontGrabberMotor = new CANSparkMax(15, MotorType.kBrushless);
        backGrabberMotor = new CANSparkMax(16, MotorType.kBrushless);
    }

    public void defaultControl(double grabberSpeed) {
        frontGrabberMotor.set(grabberSpeed);
        backGrabberMotor.set(-grabberSpeed);
    }

    public Command autoGrabberOnCommand() {
        return runOnce(() -> {
            frontGrabberMotor.set(-1);
            backGrabberMotor.set(1);
        });
    }

    public Command autoGrabberOutCommand() {
        return runOnce(() -> {
            frontGrabberMotor.set(1);
            backGrabberMotor.set(-1);
        });
    }

    public Command autoGrabberOffCommand() {
        return runOnce(() -> {
            frontGrabberMotor.set(0);
            backGrabberMotor.set(0);
        });
    }
}
