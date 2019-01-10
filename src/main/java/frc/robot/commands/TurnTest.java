
package frc.robot.commands;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Direction;


public class TurnTest extends Auto {
    
    // public TurnTest() {
    //     super();
    // }

    public void auto() {
        System.out.println("David 1");
        try {
            Robot.driveTrain.turnProportionalOnMeasurement(90, Direction.CLOCKWISE, 1);

                    //Robot.driveTrain.turnP(90, Direction.CLOCKWISE, 1, 0.5, 1000000);
        }
        catch (Exception e) {
            e.printStackTrace();
        }

    }

    public void loop() {

    }

    public void stop() {
            Robot.driveTrain.stopDrive();
    }
}

 