package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier targetLockSup;
    private DoubleSupplier targetRotationSup;
    private DoubleSupplier speedControlSup;
    private DoubleSupplier gyroOffsetSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier targetLockSup,
            DoubleSupplier targetRotationSup, DoubleSupplier speedControlSup, DoubleSupplier gyroOffsetSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.targetLockSup = targetLockSup;
        this.targetRotationSup = targetRotationSup;
        this.speedControlSup = speedControlSup;
        this.gyroOffsetSup = gyroOffsetSup;
    }

    @Override
    public void execute() {
        // Get values for everything, and set
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        double targetRotation = MathUtil.applyDeadband(targetRotationSup.getAsDouble(), 0.5);
        double speedControlVal = MathUtil.applyDeadband(speedControlSup.getAsDouble(), Constants.stickDeadband);
        double gyroOffsetVal = gyroOffsetSup.getAsDouble();

        if (speedControlVal < 0.2) {
            speedControlVal = 0.2;
        }

        // And drive
        if (targetLockSup.getAsBoolean() == true) {
            rotationVal = targetRotation;
        }
        s_Swerve.setGyroOffset(gyroOffsetVal);
        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed).times(speedControlVal),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                !robotCentricSup.getAsBoolean(),
                true);
    }
}
