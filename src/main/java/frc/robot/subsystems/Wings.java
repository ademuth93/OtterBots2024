package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wings extends SubsystemBase {
    private Solenoid extendSolenoid;
    private Solenoid retractSolenoid;
    private Solenoid deploySolenoid;
    
    public Wings() {
        extendSolenoid = new Solenoid(2, PneumaticsModuleType.CTREPCM, 4);
        retractSolenoid = new Solenoid(2, PneumaticsModuleType.CTREPCM, 3);
        deploySolenoid = new Solenoid(2, PneumaticsModuleType.CTREPCM, 0);
    }

    public void defaultControl(boolean isDefenseUp, boolean areWingsUp) {
        extendSolenoid.set(isDefenseUp);
        retractSolenoid.set(isDefenseUp);
        deploySolenoid.set(areWingsUp);
    }
}
