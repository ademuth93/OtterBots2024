package frc.robot;

import frc.robot.commands.*;
import frc.robot.oi.LimeLight;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

    // Subsystems
    private final Swerve s_Swerve = new Swerve();

    // Controllers
    private final XboxController driverController = new XboxController(2);

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

    // Others
    public final Timer m_timer = new Timer();
    private final LimeLight a_limelight = new LimeLight();
    public double gyroOffset = 0.0;

    // The container for the robot. Contains subsystems, OI devices, and commands.
    public RobotContainer() {

        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driverController.getRawAxis(translationAxis),
                        () -> -driverController.getRawAxis(strafeAxis),
                        () -> -driverController.getRawAxis(rotationAxis) / 2.0,
                        () -> robotCentric.getAsBoolean(),
                        () -> targetLock.getAsBoolean(),
                        () -> a_limelight.getdegRotationToTarget(),
                        () -> driverController.getRawAxis(speedControl),
                        () -> gyroOffset));
        m_timer.start();
        configureButtonBindings();

    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> this.zeroGyro()));
    }

    public void zeroGyro() {
        s_Swerve.zeroGyro();
        gyroOffset = 0;
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
