
package frc.robot.Autonomous;
import java.util.ArrayList;

import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain.Direction;
import java.util.Arrays;


public class TurnTest extends Auto {
    
    public int currState = 0;
    public enum States {
     turn   
    }
    public ArrayList<States> statesList = new ArrayList();

    public void auto() {
        statesList.addAll(Arrays.asList(States.turn));
    }

    public void loop() {

        switch (statesList.get(currState)) {
            case turn: 
                if (Robot.driveTrain.turnProportionalOnMeasurement(90, Direction.CLOCKWISE, 1)) {
                    nextState();
                }
                break;
        }

    }

    public void stop() {
            Robot.driveTrain.stopDrive();
    }

    public void nextState() {
        currState++;
    }
}

 