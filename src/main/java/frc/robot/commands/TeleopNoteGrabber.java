package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.NoteGrabber;

public class TeleopNoteGrabber extends Command {
    private NoteGrabber sNoteGrabber;
    private DoubleSupplier noteGrabberSup;

    public TeleopNoteGrabber(NoteGrabber sNoteGrabber, DoubleSupplier noteGrabberSup) {
        this.sNoteGrabber = sNoteGrabber;
        this.noteGrabberSup = noteGrabberSup;
        addRequirements(sNoteGrabber);
    }

    @Override
    public void execute() {
        double noteGrabberValue = MathUtil.applyDeadband(noteGrabberSup.getAsDouble(), Constants.stickDeadband);
        sNoteGrabber.defaultControl(noteGrabberValue);
    }
}
