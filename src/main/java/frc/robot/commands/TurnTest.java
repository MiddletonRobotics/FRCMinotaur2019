
package frc.robot.commands;
import java.util.ArrayList;

import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Direction;
import java.util.Arrays;


public class TurnTest extends Auto {
    
    public boolean autoCompleted = false;
    public int currState = 0;
    public enum States {
     turn   
    }
    public ArrayList<States> statesList = new ArrayList();

    public void auto() {
        statesList.addAll(Arrays.asList(States.turn, States.turn,States.turn,States.turn,States.turn,States.turn,States.turn,States.turn));
    }

    public void loop() {
        if (!autoCompleted) {
            switch (statesList.get(currState)) {
                case turn: 
                    if (Robot.driveTrain.turnProportionalOnMeasurement(45, Direction.CLOCKWISE)) {
                        nextState();
                    }
                    break;
            }
        }
    }

    public void stop() {
            Robot.driveTrain.stopDrive();
    }

    public void nextState() {
        currState++;
        if (currState >= statesList.size()) {
            autoCompleted = true;
        }
    }
}

 