/* Abbreviations Guide
 * Formatting - not mine to deal with
 */

package frc.lib.util;

import com.ctre.phoenix6.controls.DutyCycleOut;

/** Sets status frames for the CTRE CANCoder. */
public class CANCoderUtil {
    public enum CCUsage {
        kAll,
        kMinimal
    }

    /**
     * This function allows reducing a CANCoder's CAN bus utilization by reducing
     * the periodic status
     * frame period of nonessential frames from 10ms to 255ms.
     *
     * <p>
     * See https://docs.ctre-phoenix.com/en/stable/ch18_CommonAPI.html#cancoder for
     * a description
     * of the status frames.
     *
     * @param cancoder The CANCoder to adjust the status frames on.
     * @param usage    The status frame feedback to enable. kAll is the default when
     *                 a CANCoder
     *                 isconstructed.
     */

    final DutyCycleOut mRequest = new DutyCycleOut(0);

    public static void setCANCoderBusUsage(CCUsage usage, DutyCycleOut mRequest) {
        if (usage == CCUsage.kAll) {
            mRequest.UpdateFreqHz = 100;
        } else if (usage == CCUsage.kMinimal) {
            mRequest.UpdateFreqHz = 10;
        }
    }
}