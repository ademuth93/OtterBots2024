package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.pathplanner.lib.util.PIDConstants;

import frc.robot.commands.*;
import frc.robot.oi.LimeLight;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

    // PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example Path", new
    // PathConstrains(4, 3));

    // Subsystems
    private final Swerve s_Swerve = new Swerve();
    private final LimelightVision a_LimelightVision = new LimelightVision();
    private final NoteGrabber sNoteGrabber = new NoteGrabber();
    private final Shooter sShooter = new Shooter();
    private final Lift sLift = new Lift();
    private final Climber sClimber = new Climber();

    // Controllers
    private final XboxController driverController = new XboxController(2);
    private final XboxController operatorController = new XboxController(3);

    // Driver Controls
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int speedControl = XboxController.Axis.kRightTrigger.value;

    // Driver Buttons
    private final JoystickButton robotCentric = new JoystickButton(driverController,
            XboxController.Button.kLeftBumper.value);
    private final JoystickButton zeroGyro = new JoystickButton(driverController, XboxController.Button.kY.value);
    private final JoystickButton targetLock = new JoystickButton(driverController, XboxController.Button.kA.value);

    // Operator Controls
    private final int noteGrabberSpeed = XboxController.Axis.kLeftY.value;
    private final int shooterSpeed = XboxController.Axis.kRightY.value;
    private final int leftClimberSpeed = XboxController.Axis.kLeftTrigger.value;
    private final int rightClimberSpeed = XboxController.Axis.kRightTrigger.value;

    // Operator Buttons
    private final JoystickButton leftClimberDown = new JoystickButton(operatorController,
            XboxController.Button.kLeftBumper.value);
    private final JoystickButton rightClimberDown = new JoystickButton(operatorController,
            XboxController.Button.kRightBumper.value);
    private final JoystickButton liftExtender = new JoystickButton(operatorController, XboxController.Button.kB.value);
    private final JoystickButton liftRetractor = new JoystickButton(operatorController, XboxController.Button.kA.value);

    // Autos : none
    private final SendableChooser<Command> autoChooser;

    // Others
    public final Timer m_timer = new Timer();
    private final LimeLight a_limelight = new LimeLight();
    public double gyroOffset = 0.0;

    // The container for the robot. Contains subsystems, OI devices, and commands.
    public RobotContainer() {

        NamedCommands.registerCommand("NoteGrabberOn", sNoteGrabber.autoGrabberOnCommand());
        NamedCommands.registerCommand("NoteGrabberOut", sNoteGrabber.autoGrabberOutCommand());
        NamedCommands.registerCommand("NoteGrabberOff", sNoteGrabber.autoGrabberOffCommand());
        NamedCommands.registerCommand("Extend", sLift.autoRaiseCommand());
        NamedCommands.registerCommand("Retract", sLift.autoLowerCommand());
        NamedCommands.registerCommand("ShooterOut", sShooter.autoShooterOutCommand());
        NamedCommands.registerCommand("ShooterIn", sShooter.autoShooterInCommand());
        NamedCommands.registerCommand("ShooterOff", sShooter.autoShooterOffCommand());

        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driverController.getRawAxis(translationAxis),
                        () -> -driverController.getRawAxis(strafeAxis),
                        () -> -driverController.getRawAxis(rotationAxis) / 2 / 9 * 6.75,
                        () -> robotCentric.getAsBoolean(),
                        () -> targetLock.getAsBoolean(),
                        () -> a_limelight.getdegRotationToTarget(),
                        () -> driverController.getRawAxis(speedControl),
                        () -> gyroOffset));

        // Timer? TODO: Figure out what this is
        m_timer.start();

        configureButtonBindings();

        // Auto stuff
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        sNoteGrabber.setDefaultCommand(
                new TeleopNoteGrabber(
                        sNoteGrabber,
                        () -> operatorController.getRawAxis(noteGrabberSpeed)));

        sShooter.setDefaultCommand(
                new TeleopShooter(
                        sShooter,
                        () -> operatorController.getRawAxis(shooterSpeed) / 2.5));

        sClimber.setDefaultCommand(
                new TeleopClimber(
                        sClimber,
                        () -> operatorController.getRawAxis(leftClimberSpeed),
                        () -> operatorController.getRawAxis(rightClimberSpeed),
                        leftClimberDown,
                        rightClimberDown));

        sLift.setDefaultCommand(
                new TeleopLift(
                        sLift,
                        liftExtender,
                        liftRetractor));
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> this.zeroGyro()));
    }

    public void zeroGyro() {
        s_Swerve.zeroGyro();
        gyroOffset = 0;
    }

    public void setPipeline(int pipeline) {
        a_LimelightVision.setPipeline(pipeline);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
