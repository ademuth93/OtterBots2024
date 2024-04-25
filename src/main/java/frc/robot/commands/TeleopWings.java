package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wings;

public class TeleopWings extends Command {
    private Wings sWings;
    private BooleanSupplier buttonUp;
    private BooleanSupplier buttonDown;

    private boolean lastState;
    private boolean defenseUp;
    private boolean defenseDown;

    public TeleopWings(Wings sWings, BooleanSupplier buttonUp, BooleanSupplier buttonDown) {
        this.sWings = sWings;
        this.buttonUp = buttonUp;
        this.buttonDown = buttonDown;

        this.lastState = false;

        addRequirements(sWings);
    }

    @Override
    public void execute() {
        defenseUp = buttonUp.getAsBoolean();
        defenseDown = buttonDown.getAsBoolean();

        if (defenseUp) {
            lastState = true;
            sWings.defaultExtendControl(true);
            Timer.delay(5);
            sWings.defaultDeployControl(true);
        } else if (defenseDown) {
            lastState = false;
            sWings.defaultDeployControl(false);
            Timer.delay(5);
            sWings.defaultExtendControl(false);
        } else {
            sWings.defaultExtendControl(lastState);
            sWings.defaultDeployControl(lastState);
        }
    }
}
