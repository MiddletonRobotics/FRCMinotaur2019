package frc.robot.Autonomous.StateMachine;

import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

import java.util.ArrayList;

public class StateMachineRunner {

    ArrayList<AutoStates> states;
    public int currState = 0;

    public StateMachineRunner() {

    }

    public StateMachineRunner(ArrayList states) {
        this.states = states;
    }

    public void runStateMachine() {
        switch (states.get(currState)) {
            case turn90Clockwise:
                if (Robot.driveTrain.turnProportionalOnMeasurement(90, DriveTrain.Direction.CLOCKWISE, 1)) {
                    nextState();
                }
                break;
            default:
                System.out.println("Auto done");
                break;
        }
    }

    public void setStates (ArrayList states) {
        this.states = states;
    }

    public void nextState() {
        currState++;
    }
}
