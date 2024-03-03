package frc.robot.oi;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LimeLight {

    public String m_tableName;
    public NetworkTable m_table;
    private Translation3d tl3d = new Translation3d();
    private Rotation3d r3d = new Rotation3d();
    private Transform3d tf3d = new Transform3d();

    public LimeLight() {

        m_tableName = "limelight";
        m_table = NetworkTableInstance.getDefault().getTable(m_tableName);
        _heartBeat.startPeriodic(_heartBeatPeriod);
    }

    /**
     * If you changed the name of your Lime Light tell Me the New Name
     */
    public LimeLight(String tableName) {
        m_tableName = tableName;
        m_table = NetworkTableInstance.getDefault().getTable(m_tableName);
        _heartBeat.startPeriodic(_heartBeatPeriod);

    }

    /**
     * Send an instance of the NetworkTabe
     */
    public LimeLight(NetworkTable table) {
        m_table = table;
        _heartBeat.startPeriodic(_heartBeatPeriod);
    }

    public Notifier _heartBeat = new Notifier(new PeriodicRunnable());

    private CamMode currentCamMode = CamMode.kdriver;

    private StreamType currentStreamType = StreamType.kStandard;

    public double _heartBeatPeriod = 0.5;

    private LedMode currentLedMode = LedMode.kpipeLine;

    // private int ledModeSize = 4;

    public boolean connected;

    public enum CamMode {
        kvision,
        kdriver;
    }

    public enum StreamType {
        kStandard,
        kPiPMain,
        kPiPSecondary;
    }

    public enum LedMode {
        kpipeLine, // 0 use the LED Mode set in the current pipeline
        kforceOff, // 1 force off
        kforceBlink, // 2 force blink
        kforceOn; // 3 force on

    }

    class PeriodicRunnable implements java.lang.Runnable {

        private int testCount;
        private int testCountLimitCheck = 3;

        private double hbCount;

        private double lasthbcount;

        public void run() {

            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }

            hbCount = getHeartbeat();
            if (hbCount == lasthbcount) {
                testCount++;
            } else {
                lasthbcount = hbCount;
                testCount = 0;
                connected = true;
            }

            if (testCount > testCountLimitCheck) {
                connected = false;
            }

        }

    }

    /** */
    public String getName() {
        return m_tableName;
    }

    /** */
    public boolean isConnected() {
        return connected;
    }

    /** */
    public double getHeartbeat() {
        NetworkTableEntry tl = m_table.getEntry("hb");
        double hb = tl.getDouble(0.0);
        return hb;
    }

    /**
     * LedMode Sets limelight’s LED state
     * 
     * kon
     * koff
     * kblink
     * 
     * @param ledMode
     */
    public void setLEDMode(LedMode ledMode) {
        currentLedMode = ledMode;
        m_table.getEntry("ledMode").setValue(ledMode.ordinal());
    }

    /**
     * Returns current LED mode of the Lime Light
     * 
     * @return LedMode
     */
    public LedMode getLEDMode() {
        int lmode = (int) m_table.getEntry("ledMode").getDouble(0.);
        switch (lmode) {
            case 0:
                currentLedMode = LedMode.kpipeLine;
                break;
            case 1:
                currentLedMode = LedMode.kforceOff;
                break;
            case 2:
                currentLedMode = LedMode.kforceBlink;
                break;
            case 3:
                currentLedMode = LedMode.kforceOn;
                break;
            default:
                currentLedMode = LedMode.kpipeLine;
                return currentLedMode;

        }

        return currentLedMode;
    }

    /**
     * camMode Sets limelight’s operation mode
     * 
     * kvision
     * kdriver (Increases exposure, disables vision processing)
     * 
     * @param camMode
     */

    public void setCamMode(CamMode camMode) {
        currentCamMode = camMode;

        m_table.getEntry("camMode").setValue(camMode.ordinal());
    }

    // /**
    // * Returns current Cam mode of the Lime Light
    // *
    // * @return CamMode
    // */
    public CamMode getCamMode() {
        int cmode = (int) m_table.getEntry("camMode").getDouble(0.);
        switch (cmode) {
            case 0:
                currentCamMode = CamMode.kvision;
                break;
            case 1:
                currentCamMode = CamMode.kdriver;
                break;
            default:
                currentCamMode = CamMode.kvision;
                break;
        }
        return currentCamMode;
    }

    /**
     * stream Sets limelight’s streaming mode
     * 
     * kStandard - Side-by-side streams if a webcam is attached to Limelight
     * kPiPMain - The secondary camera stream is placed in the lower-right corner of
     * the primary camera stream
     * kPiPSecondary - The primary camera stream is placed in the lower-right corner
     * of the secondary camera stream
     * 
     * @param stream
     */
    public void setStream(StreamType streamType) {
        currentStreamType = streamType;
        m_table.getEntry("stream").setValue(streamType.ordinal());
    }

    /** */
    public StreamType getStream() {
        int smode = (int) m_table.getEntry("stream").getDouble(0.);
        switch (smode) {
            case 0:
                currentStreamType = StreamType.kStandard;
                break;
            case 1:
                currentStreamType = StreamType.kPiPMain;
                break;
            case 2:
                currentStreamType = StreamType.kPiPSecondary;
                break;
            default:
                currentStreamType = StreamType.kStandard;
                break;
        }

        return currentStreamType;
    }

    /**
     * pipeline Sets limelight’s current pipeline
     * 
     * 0 . 9 Select pipeline 0.9
     * 
     * @param pipeline
     */

    public void setPipeline(int pipeline) {
        if (pipeline < 0) {
            pipeline = 0;
            throw new IllegalArgumentException("Pipeline can not be less than zero");
        } else if (pipeline > 9) {
            pipeline = 9;
            throw new IllegalArgumentException("Pipeline can not be greater than nine");
        }
        m_table.getEntry("pipeline").setValue(pipeline);
    }

    /**
     * Returns current Pipeline of the Lime Light
     * 
     * @return Pipelinge
     */
    public int getPipeline() {
        NetworkTableEntry pipeline = m_table.getEntry("pipeline");
        double pipe = pipeline.getInteger(0);
        return (int) pipe;
    }

    /**
     * tl The pipeline’s latency contribution (ms) Add at least 11ms for image
     * capture latency.
     * 
     * @return
     */
    public double getPipelineLatency() {
        NetworkTableEntry tl = m_table.getEntry("tl");
        double l = tl.getDouble(0.0);
        return l;
    }

    /**
     * tl Reset the pipeline’s latency contribution/**
     * tl The pipeline’s latency contribution (ms) Add at least 11ms for image
     * capture latency.
     * 
     * @return
     *
     * 
     * @return
     */

    public void resetPipelelineLatency() {
        m_table.getEntry("tl").setValue(0.0);
    }

    public void takeSnapshot(int on) {
        NetworkTableEntry sn = m_table.getEntry("snapshot");
        sn.setValue(on);
    }

    /**
     * tv Whether the limelight has any valid targets (0 or 1)
     * 
     * @return
     */
    public boolean getIsTargetFound() {

        NetworkTableEntry tv = m_table.getEntry("tv");

        double v = tv.getDouble(0.);

        return v == 1.0;
    }

    /**
     * tx Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
     * 
     * @return
     */
    public double getdegRotationToTarget() {
        NetworkTableEntry tx = m_table.getEntry("tx");
        double x = tx.getDouble(0.0);
        return x;

    }

    public double getRY() {
        NetworkTableEntry TRY = m_table.getEntry("targetpose_cameraspace");
        double[] RY = TRY.getDoubleArray(new double[6]); // {1.0, 2.0, 3.0, 1.0, 0.5, 56.0}
        return (RY[4]);
    }

    /**
     * ty Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
     * 
     * @return
     */
    public double getdegVerticalToTarget() {
        NetworkTableEntry ty = m_table.getEntry("ty");
        double y = ty.getDouble(0.0);
        return y;
    }

    /**
     * ta Target Area (0% of image to 100% of image)
     * 
     * @return
     */
    public double getTargetArea() {
        NetworkTableEntry ta = m_table.getEntry("ta");
        double a = ta.getDouble(0.0);
        return a;
    }

    /**
     * ts Skew or rotation (-90 degrees to 0 degrees)
     * 
     * @return
     */
    public double getSkew_Rotation() {
        NetworkTableEntry ts = m_table.getEntry("ts");
        double s = ts.getDouble(0.0);
        return s;
    }

    /**
     * Combined bounding box width
     * 
     * @return
     */
    public double getBoundingBoxWidth() {
        NetworkTableEntry thor = m_table.getEntry("thor");
        double t = thor.getDouble(0.0);
        return t;
    }

    /**
     * Combined bounding box height
     * 
     * @return
     */
    public double getBoundingBoxHeight() {
        NetworkTableEntry tvert = m_table.getEntry("tvert");
        double t = tvert.getDouble(0.0);
        return t;
    }

    /**
     * tshort Shortest side of fitted bounding box
     * 
     * @return
     */
    public double getBoundingBoxShortestSide() {
        NetworkTableEntry tsh = m_table.getEntry("tshort");
        double sh = tsh.getDouble(0.0);
        return sh;
    }

    /**
     * tlong Longest side of fitted bounding box
     * 
     * @return
     */
    public double getBoundingBoxLongestSide() {
        NetworkTableEntry tlng = m_table.getEntry("tlong");
        double l = tlng.getDouble(0.0);
        return l;
    }

    // *************** Advanced Usage with Raw Contours *********************

    /**
     * Limelight posts three raw contours to NetworkTables that are not influenced
     * by your grouping mode.
     * That is, they are filtered with your pipeline parameters, but never grouped.
     * X and Y are returned
     * in normalized screen space (-1 to 1) rather than degrees. *
     */

    public double getAdvanced_RotationToTarget(int n) {
        String nt = "tx" + String.valueOf(n);
        NetworkTableEntry txRaw = m_table.getEntry(nt);
        double x = txRaw.getDouble(0.0);
        return x;
    }

    /** */
    public double getAdvanced_degVerticalToTarget(int n) {
        String nt = "ty" + String.valueOf(n);
        NetworkTableEntry tyRaw = m_table.getEntry(nt);
        double y = tyRaw.getDouble(0.0);
        return y;
    }

    /** */
    public double getAdvanced_TargetArea(int n) {
        String nt = "ta" + String.valueOf(n);
        NetworkTableEntry taRaw = m_table.getEntry(nt);
        double a = taRaw.getDouble(0.0);
        return a;
    }

    /** */
    public double getAdvanced_Skew_Rotation(int n) {
        String nt = "ts" + String.valueOf(n);
        NetworkTableEntry tsRaw = m_table.getEntry(nt);
        double s = tsRaw.getDouble(0.0);
        return s;
    }

    // Raw Crosshairs:
    // If you are using raw targeting data, you can still utilize your calibrated
    // crosshairs:

    public double getAdvanced_RawCrosshair_X(int n) {
        String nt = "cx" + String.valueOf(n);
        NetworkTableEntry cxRaw = m_table.getEntry(nt);
        double x = cxRaw.getDouble(0.0);
        return x;
    }

    /** */
    public double getAdvanced_RawCrosshair_Y(int n) {
        String nt = "cy" + String.valueOf(n);
        NetworkTableEntry cyRaw = m_table.getEntry(nt);
        double y = cyRaw.getDouble(0.0);
        return y;
    }

    /** */
    public String getSendCornersOn() {
        NetworkTableEntry value = m_table.getEntry("cornxy");
        return value.getName();
    }

    /**
     * Is only available if send corners is turned on
     * 
     * @return
     */

    public double[] getCorners() {

        double[] temp = { 0, 0, 0, 0 };// default for getEntry
        NetworkTableEntry value = m_table.getEntry("cornxy");
        double[] result = value.getDoubleArray(temp);
        if (result.length == 4)
            return result;
        else
            return temp;

    }

    /**
     * Returns ID of the April Tag
     * 
     * @return Pipelinge
     */

    public int getAprilTagID() {
        NetworkTableEntry id = m_table.getEntry("tid");
        int value = (int) id.getDouble(0.0f);
        return value;
    }

    /**
     * Returns pose of the April Tag
     * 
     * @return Transform3d
     */

    public Transform3d getRobotTransform()

    {

        double[] temp = { 0, 0, 0, 0, 0, 0 };// default for getEntry
        NetworkTableEntry value = m_table.getEntry("botpose");

        double[] result = value.getDoubleArray(temp);

        if (result.length == 6) {

            tl3d = new Translation3d(result[0], result[1], result[2]);
            for (int i = 3; i > 5; i++) {
                result[i] = Units.degreesToRadians(result[i]);
            }

            r3d = new Rotation3d(result[3], result[4], result[5]);
            tf3d = new Transform3d(tl3d, r3d);
        } else
            tf3d = new Transform3d();
        return tf3d;
    }

    /**
     * camtran Camera transform in target space of primary apriltag or solvepnp
     * target.
     * NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)
     * 
     * @return
     */
    public Transform3d getCamTran() {
        double[] temp = { 0, 0, 0, 0, 0, 0 };// default for getEntry
        NetworkTableEntry value = m_table.getEntry("camtran");
        double[] result = value.getDoubleArray(temp);
        tl3d = new Translation3d(result[0], result[1], result[2]);
        for (int i = 3; i > 5; i++) {
            result[i] = Units.degreesToRadians(result[i]);
        }
        r3d = new Rotation3d(result[3], result[4], result[5]);
        tf3d = new Transform3d(tl3d, r3d);

        return tf3d;

    }

    /**
     * Returns class of the detected object
     * 
     * @return Pipelinge
     */

    public int getNeuralClassID() {
        NetworkTableEntry id = m_table.getEntry("tclass");
        int value = (int) id.getDouble(0.0f);
        return value;
    }

    /** */
    public double[] getAveCrossHairColor() {
        double[] temp = { 0, 0, 0 };
        NetworkTableEntry value = m_table.getEntry("tc");
        double[] hsv = value.getDoubleArray(temp);
        return hsv;

    }

    /**
     * 
     * @param sizes
     * 
     *              * crop sets the 4 sides of the crop rectange
     * 
     *              returns fail if parameter array size is not 4
     * 
     *              returns fail if crop values outside range +1 to -1
     * 
     * 
     * @return fail
     */

    public boolean setCropRectangle(double[] sizes) {
        boolean fail = false;
        fail = sizes.length != 4;
        if (!fail) {
            for (int i = 0; i < 4; i++) {
                fail = sizes[i] > 1 || sizes[i] < -1;
                if (fail)
                    break;
            }
            if (!fail)

                m_table.getEntry("crop").setDoubleArray(sizes);

        }
        return fail;
    }

    /**
     * 
     * 
     * @return
     */
    public Command GetSnapShot() {
        return new SequentialCommandGroup(new InstantCommand(() -> takeSnapshot(1)),
                new WaitCommand(.1),
                new InstantCommand(() -> takeSnapshot(0)));
    }

    public Command ChangeStreamType(StreamType type) {
        return new InstantCommand(() -> setStream(type));
    }

    public Command ChangePipeline(int n) {
        return new InstantCommand(() -> setPipeline(n)); // 0-9
    }

    public Command ToggleCamMode() {

        return new ConditionalCommand(
                new InstantCommand(() -> setCamMode(CamMode.kdriver)),
                new InstantCommand(() -> setCamMode(CamMode.kvision)),
                () -> (getCamMode() == CamMode.kvision));
    }

    public Command ChangeCropRectangle(double[] sizes) {

        return new InstantCommand(() -> setCropRectangle(sizes));
    }

    public Command ChangeLEDMode(LedMode mode) {
        return new InstantCommand(() -> setLEDMode(mode));
    }

}
