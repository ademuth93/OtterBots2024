package frc.lib.config;

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

        // In Phoenix 6, Talon FX and CANcoder sensors are always initialized to their absolute position.
        // In Phoenix 6, CANcoder does not support setting a custom sensor coefficient, unit string, and sensor time base.
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magnetSensorConfigs.SensorDirection = Constants.Swerve.canCoderInvert;
        magnetSensorConfigs.MagnetOffset = 0.0;
        swerveCanCoderConfig.withMagnetSensor(magnetSensorConfigs);
    }
}