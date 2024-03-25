package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    private final AHRS gyro;
    private double gyroOffset;

    public Swerve() {
        gyro = new AHRS(SPI.Port.kMXP);
        gyroOffset = 0.0;
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYawRot2d(), getModulePositions());
    }

    // Method called in TeleopSwerve to drive in teleop
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYawRot2d())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    // Resets Odometry
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYawRot2d(), getModulePositions(), pose);
    }

    // Gets module states
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    // Gets module positions
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    // Zeros the gyro
    public void zeroGyro() {
        gyro.zeroYaw();
    }

    // Sets the gyro offset
    public void setGyroOffset(double gyroOffset) {
        this.gyroOffset = gyroOffset;
    }

    // Gets the yaw
    public Rotation2d getYawRot2d() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw() + gyroOffset)
                : Rotation2d.fromDegrees(gyro.getYaw() + gyroOffset);
    }

    public double getYawDeg() {
        return gyro.getYaw();
    }

    // Resets the modules to absolute
    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYawRot2d(), getModulePositions());

        SmartDashboard.putNumber("Encoder Reading FL", mSwerveMods[0].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Encoder Reading FR", mSwerveMods[1].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Encoder Reading BL", mSwerveMods[2].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Encoder Reading BR", mSwerveMods[3].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Robot Yaw2", this.getYawDeg());

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + "Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + "Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + "Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}
