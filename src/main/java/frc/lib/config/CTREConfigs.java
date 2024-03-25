package frc.lib.config;

// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
//import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
//import com.ctre.phoenix.sensors.AbsoluteSensorRange;
//import com.ctre.phoenix.sensors.CANCoderConfiguration;
// import com.ctre.phoenix.sensors.SensorInitializationStrategy;
// import com.ctre.phoenix.sensors.SensorTimeBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import frc.robot.Constants;

public final class CTREConfigs {
    public static CurrentLimitsConfigs currentLimitsConfigs;
    public static Slot0Configs slot0Configs;
    public static OpenLoopRampsConfigs openLoopRampsConfigs;
    public static ClosedLoopRampsConfigs closedLoopRampsConfigs;
    public static TalonFXConfiguration swerveAngleFXConfig;
    public static TalonFXConfiguration swerveDriveFXConfig;

    public static MagnetSensorConfigs magnetSensorConfigs;
    public static CANcoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        currentLimitsConfigs = new CurrentLimitsConfigs();
        slot0Configs = new Slot0Configs();
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();
        magnetSensorConfigs = new MagnetSensorConfigs();

// var fx_cfg = new TalonFXConfiguration();
// // fetch *all* configs currently applied to the device
//m_motor.getConfigurator().refresh(fx_cfg);

        /* Swerve Angle Motor Configurations */
        // SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
        //     SwerveConstants.Swerve.angleEnableCurrentLimit, 
        //     SwerveConstants.Swerve.angleContinuousCurrentLimit, 
        //     SwerveConstants.Swerve.anglePeakCurrentLimit, 
        //     SwerveConstants.Swerve.anglePeakCurrentDuration);

        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimit = Constants.Swerve.angleContinuousCurrentLimit;
        currentLimitsConfigs.SupplyCurrentThreshold = 40;
        currentLimitsConfigs.SupplyTimeThreshold = 0.1;
        slot0Configs.withKP(Constants.Swerve.angleKP);
        slot0Configs.withKI(Constants.Swerve.angleKI);
        slot0Configs.withKD(Constants.Swerve.angleKD);
        slot0Configs.withKV(Constants.Swerve.angleKFF);
        swerveAngleFXConfig.withCurrentLimits(currentLimitsConfigs);
        swerveAngleFXConfig.withSlot0(slot0Configs);

        // swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.Swerve.angleEnableCurrentLimit;
        // swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.Swerve.angleContinuousCurrentLimit;
        // swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = SwerveConstants.Swerve.anglePeakCurrentLimit;
        // swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = SwerveConstants.Swerve.anglePeakCurrentDuration;
        // swerveAngleFXConfig.Slot0.kP = SwerveConstants.Swerve.angleKP;
        // swerveAngleFXConfig.Slot0.kI = SwerveConstants.Swerve.angleKI;
        // swerveAngleFXConfig.Slot0.kD = SwerveConstants.Swerve.angleKD;
        // swerveAngleFXConfig.Slot0.kV = SwerveConstants.Swerve.angleKF;
        // swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
 

        /* Swerve Drive Motor Configuration */
        // SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
        //     SwerveConstants.Swerve.driveEnableCurrentLimit, 
        //     SwerveConstants.Swerve.driveContinuousCurrentLimit, 
        //     SwerveConstants.Swerve.drivePeakCurrentLimit, 
        //     SwerveConstants.Swerve.drivePeakCurrentDuration);

        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimit = Constants.Swerve.driveContinuousCurrentLimit;
        currentLimitsConfigs.SupplyCurrentThreshold = 60;
        currentLimitsConfigs.SupplyTimeThreshold = 0.1;
        slot0Configs.withKP(Constants.Swerve.driveKP);
        slot0Configs.withKI(Constants.Swerve.driveKI);
        slot0Configs.withKD(Constants.Swerve.driveKD);
        slot0Configs.withKV(Constants.Swerve.driveKFF);
        openLoopRampsConfigs.withDutyCycleOpenLoopRampPeriod(Constants.Swerve.openLoopRamp);
        closedLoopRampsConfigs.withDutyCycleClosedLoopRampPeriod(Constants.Swerve.closedLoopRamp);
        swerveDriveFXConfig.withCurrentLimits(currentLimitsConfigs);
        swerveDriveFXConfig.withSlot0(slot0Configs);
        swerveDriveFXConfig.withOpenLoopRamps(openLoopRampsConfigs);
        swerveDriveFXConfig.withClosedLoopRamps(closedLoopRampsConfigs);

        // swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.Swerve.driveEnableCurrentLimit;
        // swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.Swerve.driveContinuousCurrentLimit;
        // swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = SwerveConstants.Swerve.drivePeakCurrentLimit;
        // swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = SwerveConstants.Swerve.drivePeakCurrentDuration;
        // swerveDriveFXConfig.Slot0.kP = SwerveConstants.Swerve.driveKP;
        // swerveDriveFXConfig.Slot0.kI = SwerveConstants.Swerve.driveKI;
        // swerveDriveFXConfig.Slot0.kD = SwerveConstants.Swerve.driveKD;
        // swerveDriveFXConfig.Slot0.kV = SwerveConstants.Swerve.driveKF;        
        // swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConstants.Swerve.openLoopRamp;
        // swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SwerveConstants.Swerve.closedLoopRamp;
        
        // swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        // swerveDriveFXConfig.openloopRamp = SwerveConstants.Swerve.openLoopRamp;
        // swerveDriveFXConfig.closedloopRamp = SwerveConstants.Swerve.closedLoopRamp;
        

        /* Swerve CANCoder Configuration */
        // swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        // swerveCanCoderConfig.sensorDirection = SwerveConstants.Swerve.canCoderInvert;
        // swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        // swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        
        // swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        // swerveCanCoderConfig.MagnetSensor.SensorDirection = SwerveConstants.Swerve.canCoderInvert;
        // In Phoenix 6, Talon FX and CANcoder sensors are always initialized to their absolute position.
        // In Phoenix 6, CANcoder does not support setting a custom sensor coefficient, unit string, and sensor time base.
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magnetSensorConfigs.SensorDirection = Constants.Swerve.canCoderInvert;
        magnetSensorConfigs.MagnetOffset = 0.0;
        swerveCanCoderConfig.withMagnetSensor(magnetSensorConfigs);
    }
}