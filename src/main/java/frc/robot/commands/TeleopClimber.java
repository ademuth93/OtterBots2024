package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class TeleopClimber extends Command {
    private Climber sClimber;
    private DoubleSupplier leftClimberSup;
    private DoubleSupplier rightClimberSup;
    private BooleanSupplier leftClimberDown;
    private BooleanSupplier rightClimberDown;

    public TeleopClimber(Climber sClimber, DoubleSupplier leftClimberSup, DoubleSupplier rightClimberSup,
            BooleanSupplier leftClimberDown, BooleanSupplier rightClimberDown) {
        this.sClimber = sClimber;
        this.leftClimberSup = leftClimberSup;
        this.rightClimberSup = rightClimberSup;
        this.leftClimberDown = leftClimberDown;
        this.rightClimberDown = rightClimberDown;
        addRequirements(sClimber);
    }

    @Override
    public void execute() {
        double leftClimberValue;
        double rightClimberValue;

        if (leftClimberDown.getAsBoolean()) {
            leftClimberValue = -1;
        } else if (leftClimberSup.getAsDouble() > 0.1) {
            leftClimberValue = leftClimberSup.getAsDouble();
        } else {
            leftClimberValue = 0;
        }

        if (rightClimberDown.getAsBoolean()) {
            rightClimberValue = -1;
        } else if (rightClimberSup.getAsDouble() > 0.1) {
            rightClimberValue = rightClimberSup.getAsDouble();
        } else {
            rightClimberValue = 0;
        }

        sClimber.defaultControl(leftClimberValue, rightClimberValue);
    }
}
