
package frc.robot.Utilities;

import frc.robot.Utilities.Constants.Constants;

public class Utils implements Constants {

    public static void sleep(int ms) {
        try {
            Thread.sleep(ms);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
