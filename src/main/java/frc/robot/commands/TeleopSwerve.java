package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private Swerve sSwerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private DoubleSupplier speedControlSup;
    private BooleanSupplier orientToTagSup;
    private DoubleSupplier gyroOffsetSup;

    public TeleopSwerve(Swerve sSwerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup,
            DoubleSupplier speedControlSup, BooleanSupplier orientToTagSup, DoubleSupplier gyroOffsetSup) {
        this.sSwerve = sSwerve;
        addRequirements(sSwerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.speedControlSup = speedControlSup;
        this.orientToTagSup = orientToTagSup;
        this.gyroOffsetSup = gyroOffsetSup;
    }

    @Override
    public void execute() {
        // Get values for everything, and set
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        double speedControlVal = MathUtil.applyDeadband(speedControlSup.getAsDouble(), Constants.stickDeadband);
        boolean orientToTagVal = orientToTagSup.getAsBoolean();
        double gyroOffsetVal = gyroOffsetSup.getAsDouble();

        if (speedControlVal < 0.2) {
            speedControlVal = 0.2;
        }

        sSwerve.setGyroOffset(gyroOffsetVal);

        if(orientToTagVal) {
            sSwerve.driveVision(speedControlVal, !robotCentricSup.getAsBoolean(), true);

        }
        else {
            sSwerve.drive(
                    new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), // .times(speedControlVal)
                    rotationVal * Constants.Swerve.maxAngularVelocity,
                    !robotCentricSup.getAsBoolean(),
                    true);
        }
    }
}
