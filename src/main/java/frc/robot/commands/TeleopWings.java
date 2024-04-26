package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wings;

public class TeleopWings extends Command {
    private Wings sWings;

    private BooleanSupplier deployUp;
    private BooleanSupplier deployDown;
    private BooleanSupplier wingsUp;
    private BooleanSupplier wingsDown;

    private boolean defenseLastState;
    private boolean wingsLastState;
    private boolean setDefense;
    private boolean setWings;

    public TeleopWings(Wings sWings, BooleanSupplier deployUp, BooleanSupplier deployDown, BooleanSupplier wingsUp, BooleanSupplier wingsDown) {
        this.sWings = sWings;
        this.deployUp = deployUp;
        this.deployDown = deployDown;
        this.wingsUp = wingsUp;
        this.wingsDown = wingsDown;

        this.defenseLastState = false;
        this.wingsLastState = false;
        this.setDefense = false;
        this.setWings = false;

        addRequirements(sWings);
    }

    @Override
    public void execute() {
        if(deployUp.getAsBoolean()) {
            setDefense = true;
            defenseLastState = true;
        }
        else if(deployDown.getAsBoolean()) {
            setDefense = false;
            defenseLastState = false;
        }
        else {
            setDefense = defenseLastState;
        }

        if(wingsUp.getAsBoolean()) {
            setWings = true;
            wingsLastState = true;
        }
        else if(wingsDown.getAsBoolean()) {
            setWings = false;
            wingsLastState = false;
        }
        else {
            setWings = wingsLastState;
        }
        sWings.defaultControl(setDefense, setWings);
    }
}
