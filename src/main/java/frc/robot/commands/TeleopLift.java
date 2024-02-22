package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;

public class TeleopLift extends Command {
    private Lift sLift;
    BooleanSupplier buttonLiftUp;
    BooleanSupplier buttonLiftDown;
    private boolean lastState;
    boolean up;
    boolean down;

    public TeleopLift(Lift sLift, BooleanSupplier buttonLiftUp, BooleanSupplier buttonLiftDown) {
        this.sLift = sLift;
        this.buttonLiftUp = buttonLiftUp;
        this.buttonLiftDown = buttonLiftDown;

        this.lastState = false;

        addRequirements(sLift);
    }

    @Override
    public void execute() {
        up = buttonLiftUp.getAsBoolean();
        down = buttonLiftDown.getAsBoolean();

        if (down) {
            sLift.defaultControl(false);
            lastState = false;
        } else if (up) {
            sLift.defaultControl(true);
            lastState = true;
        } else {
            sLift.defaultControl(lastState);
        }

    }
}
