
package frc.robot;

public class Utils implements Constants {

    public static void sleep(int ms) {
        try {
            Thread.sleep(ms);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static double iPhoneMath(double percentHeight) {
        return LIFT_BOTTOM + SENT_FROM_MY_iPHONE * percentHeight;
    }

    public static void resetRobot() {
        Robot.driveTrain.reset();
//		Robot.speedShift.reset();
//		Robot.scalerShift.reset();
//		Robot.gearGrabber.reset();
        Robot.lift.reset();
        Robot.intake.reset();
//		Robot.shootaur.reset();
        Robot.intakeTilt.reset();
        Robot.driveTrain.resetGyro();
//		Robot.driveTrain.getLeftTalon().setEncPosition(0);
//		Robot.driveTrain.getRightTalon().setEncPosition(0);
    }

}
