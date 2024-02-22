package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class TeleopShooter extends Command {
    private Shooter sShooter;
    private DoubleSupplier shooterSup;

    public TeleopShooter(Shooter sShooter, DoubleSupplier shooterSup) {
        this.sShooter = sShooter;
        this.shooterSup = shooterSup;
        addRequirements(sShooter);
    }

    @Override
    public void execute() {
        double shooterValue = MathUtil.applyDeadband(shooterSup.getAsDouble(), Constants.stickDeadband);
        sShooter.defaultControl(shooterValue);
    }
}
