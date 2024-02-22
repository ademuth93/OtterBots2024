package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {
    private Solenoid topSolenoid;
    private Solenoid bottomSolenoid;

    public Lift() {
        topSolenoid = new Solenoid(2, PneumaticsModuleType.CTREPCM, 1);
        bottomSolenoid = new Solenoid(2, PneumaticsModuleType.CTREPCM, 2);
    }

    public void defaultControl(boolean isLiftUp) {
        topSolenoid.set(isLiftUp);
        bottomSolenoid.set(isLiftUp);
    }

    public Command autoRaiseCommand() {
        return runOnce(() -> {
            topSolenoid.set(true);
            bottomSolenoid.set(true);
        });
    }

    public Command autoLowerCommand() {
        return runOnce(() -> {
            topSolenoid.set(false);
            bottomSolenoid.set(false);
        });
    }
}
