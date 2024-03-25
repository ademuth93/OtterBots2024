package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.lib.util.CANSparkMaxUtil;

public class SwerveModule {

    // defining variables
    public int moduleNumber;
    private double angleOffset;
    private Rotation2d lastAngle;

    // The two motors in a module
    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    // The two encoders in each motor and the CANCoder
    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    public CANcoder angleEncoder;

    // PIDControllers within the NEO, I think
    private final SparkPIDController driveController;
    private final SparkPIDController angleController;

    // general Configuration withing the CANCoder
    public CANcoderConfiguration swerveCanCoderConfig;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV,
            Constants.Swerve.driveKA);

    // Whenever a SwerveModule is created, this is what is called
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        swerveCanCoderConfig = new CANcoderConfiguration();

        // Angle Encoder Config
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        // Angle Motor Config
        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        configAngleMotor();

        // Drive Motor Config
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.set(percentOutput);
        } else {
            driveController.setReference(
                    desiredState.speedMetersPerSecond,
                    ControlType.kVelocity,
                    0,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle;

        // comment out ↓↓
        // Rotation2d oldAngle = getAngle();
        // angle = optimizeTurn(oldAngle, angle);
        // comment out ↑↑

        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    public double makePositiveDegrees(double angle) {
        // Take arbitrary angle and calculate its equvalent in 0-360 degree range
        double degrees = angle;
        degrees = degrees % 360;
        if (degrees < 0.0) {
            degrees += 360.;
        }
        return degrees;
    }

    public double makePositiveDegrees(Rotation2d angle) {
        return makePositiveDegrees(angle.getDegrees());
    }

    public Rotation2d optimizeTurn(Rotation2d oldAngle, Rotation2d newAngle) {
        double steerAngle = makePositiveDegrees(newAngle);
        steerAngle %= 360;
        if (steerAngle < 0.0) {
            steerAngle += 360;
        }

        double difference = steerAngle - oldAngle.getDegrees();

        // Change the target angle so the difference is in the range [-360, 360) instead of [0, 360)
        if (difference >= 360) {
            steerAngle -= 360;
        } else if (difference < -360) {
            steerAngle += 360;
        }
        difference = steerAngle - oldAngle.getDegrees();    
        
        // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
        // movement of the module is less than 90 deg
        if (difference > 90 || difference < -90) {
            steerAngle += 180;
        }

        return Rotation2d.fromDegrees(makePositiveDegrees(steerAngle));
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble() * 360.0);
    }

    public void resetToAbsolute() {
        // double absolutePosition = getCanCoder().getDegrees() - angleOffset;
        double absolutePosition = makePositiveDegrees(getCanCoder().getDegrees() - angleOffset);
        integratedAngleEncoder.setPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        // angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        angleEncoder.getConfigurator().apply(swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
        angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        angleMotor.setInverted(Constants.Swerve.angleInvert);
        angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
        integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
        angleController.setP(Constants.Swerve.angleKP);
        angleController.setI(Constants.Swerve.angleKI);
        angleController.setD(Constants.Swerve.angleKD);
        angleController.setFF(Constants.Swerve.angleKFF);
        angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        angleMotor.burnFlash();
        resetToAbsolute();
    }

    private void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        driveMotor.setInverted(Constants.Swerve.driveInvert);
        driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
        driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
        driveController.setP(Constants.Swerve.angleKP);
        driveController.setI(Constants.Swerve.angleKI);
        driveController.setD(Constants.Swerve.angleKD);
        driveController.setFF(Constants.Swerve.angleKFF);
        driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
        // return new SwerveModuleState(driveEncoder.getVelocity(), getCanCoder());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
        // return new SwerveModulePosition(driveEncoder.getPosition(), getCanCoder());
    }

    public double getAbsoluteEncoderRad() {
        double angle = angleEncoder.getPosition().getValueAsDouble();
        // double angle = angleEncoder.getAbsolutePosition().getValueAsDouble();
        // angle *= 2.0 * Math.PI / 360.0;
        angle -= angleOffset;
        return angle;
    }
}