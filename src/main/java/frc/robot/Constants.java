package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final class Swerve {
        // Drivetrain Constants
        public static final double trackWidth = Units.inchesToMeters(18.5);
        public static final double wheelBase = Units.inchesToMeters(21);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheecCircumference = wheelDiameter * Math.PI;

        // Module 0 Constants
        public static final class Mod0 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 30;
            public static final double angleOffset = 30.5; // 35.6
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        // Module 1 Constants
        public static final class Mod1 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 31;
            public static final double angleOffset = 78.1; // 80.4
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        // Module 2 Constants
        public static final class Mod2 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 32;
            public static final double angleOffset = 115.0; // 121.6
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        // Module 3 Constants
        public static final class Mod3 {
            public static final int driveMotorID = 9;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 33;
            public static final double angleOffset = 122.6; // -125.6
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.Clockwise_Positive;

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        // Drive Motor Characterization Values
        public static final double driveKS = 0.667;
        public static final double driveKV = 2.44;
        public static final double driveKA = 0.27;

        // Swerve Current Limiting
        public static final int angleContinuousCurrentLimit = 80;
        public static final int driveContinuousCurrentLimit = 20;

        // Angle Motor PID Values
        public static final double angleKP = 0.01;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKFF = 0.0;

        // Drive Motor PID Values
        public static final double driveKP = 0.01;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKFF = 0.0;

        // Swerve Voltage Compensation
        public static final double voltageComp = 12.0;

        // Motor Inverts
        public static final boolean driveInvert = false;
        public static final boolean angleInvert = false;

        // Gear Ratios
        public static final double driveGearRatio = (7.0 / 1.0);
        public static final double angleGearRatio = (12.8 / 1.0);

        // Drive Motor Conversion Factors
        public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;

        // Swerve Profiling Values
        public static final double maxSpeed = 4.5; //
        public static final double maxAngularVelocity = 11.5;

        // Swerve Drive Kinematics Constants
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(2, 0, 0), // Translation constants
                new PIDConstants(2, 0, 0), // Rotation constants
                4.5,
                0.42, // Drive base radius (distance from center to furthest module)
                new ReplanningConfig());

        public static final ReplanningConfig replanningConfig = new ReplanningConfig(

        );

        public static final boolean invertGyro = false;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
    }

    // Stick deadband constants
    public static final double stickDeadband = 0.1;
    public static final double stickDeadband2 = 1;

    public static class VisionConstants {

        /**
         * Physical location of the camera on the robot, relative to the center of the
         * robot.
         */
        public static final Transform2d CAMERA_TO_ROBOT = new Transform2d(new Translation2d(-0.3425, 0.0),
                new Rotation2d(0.0));

        public static final Transform3d CAMERA_TO_ROBOT3 = new Transform3d();

        // 15"up and 15"forward

        private static double camHeight = Units.inchesToMeters(15);

        public static double camXFromCenter = -Units.inchesToMeters(15);

        public static String cameraName = "front";

        public static final Transform3d CAMERA_TO_ROBOT_3D = new Transform3d(
                new Translation3d(camXFromCenter, 0.0, camHeight),
                new Rotation3d());
    }

    public static class AutoConstants {
        public static final double kMaxSpeedMetersPerSecondA = 3;
        public static final double kMaxAccelerationMetersPerSecondSquaredA = 3;
        public static final double kMaxAngularSpeedRadiansPerSecondA = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquaredA = Math.PI;

        public static final double kMaxSpeedMetersPerSecondB = 1;
        public static final double kMaxAccelerationMetersPerSecondSquaredB = 1;
        public static final double kMaxAngularSpeedRadiansPerSecondB = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquaredB = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecondB, kMaxAngularSpeedRadiansPerSecondSquaredB);

    }
}
