package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonCamera;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator swervePoseEstimator;
    public SwerveModule[] mSwerveMods;
    private final AHRS gyro;
    private double gyroOffset;

    PhotonCamera camera = new PhotonCamera("photonvision");

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    Transform3d robotToCamera = new Transform3d(0.0, 0.5, 0.0, new Rotation3d(0.0, 0.0, 0.0));

    PhotonPoseEstimator photonPoseEstimator;

    PhotonPipelineResult result;

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(45.0)));

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

        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(),
                getModulePositions(), getPose());
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera, robotToCamera);

        result = camera.getLatestResult();

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                this::getSpeeds,
                this::driveRobotRelative,
                Constants.Swerve.pathFollowerConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
    }

    // Drive. Used in teleopSwerve
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public Command followPathCommand() {
        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(1.0, 1.0, 2 * Math.PI, 4 * Math.PI),
                new GoalEndState(0.0, Rotation2d.fromDegrees(45.0)));

        return new FollowPathHolonomic(
                path,
                this::getPose, // Robot pose supplier
                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(1.0, 0.0, 0.0), // Rotation PID constants
                        1.0, // Max module speed, in m/s
                        Units.inchesToMeters(14), // Drive base radius in meters. Distance from robot center to furthest
                                                  // module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    // Setting Module States in Auto, supposedly
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    // Gets pose
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    // Resets Odometry
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
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

    public ChassisSpeeds getSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    // From PathPlanner guy
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
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
    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw() + gyroOffset)
                : Rotation2d.fromDegrees(gyro.getYaw() + gyroOffset);
    }

    // Gets the roll
    public double getRoll() {
        return gyro.getRoll();
    }

    // Gets the yaw, another type
    public double getYaw2() {
        return gyro.getYaw();
    }

    // Gets the pitch
    public double getPitch() {
        return gyro.getPitch();
    }

    // Resets the modules to absolute
    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }



    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getModulePositions());
        swervePoseEstimator.update(getYaw(), getModulePositions());

        result = camera.getLatestResult();

        if(result.hasTargets()) {
            swervePoseEstimator.addVisionMeasurement(photonPoseEstimator.update().get().estimatedPose.toPose2d(),  Timer.getFPGATimestamp());//Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Capture("photonvision") + LimelightHelpers.getLatency_Pipeline("photonvision")) / 1000.0);
            SmartDashboard.putNumber("Pose from vision X", photonPoseEstimator.update().get().estimatedPose.toPose2d().getX());
        }

        SmartDashboard.putNumber("Encoder Reading FL", mSwerveMods[0].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Encoder Reading FR", mSwerveMods[1].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Encoder Reading BL", mSwerveMods[2].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Encoder Reading BR", mSwerveMods[3].getAbsoluteEncoderRad());
        SmartDashboard.putString("Robot Yaw", this.getYaw().toString());
        SmartDashboard.putNumber("Robot Yaw2", this.getYaw2());
        SmartDashboard.putNumber("Robot Roll", this.getRoll());
        SmartDashboard.putNumber("Robot Pitch", this.getPitch());

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + "Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + "Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + "Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}
