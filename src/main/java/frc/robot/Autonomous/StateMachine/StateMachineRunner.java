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
        if (states != null) {
            switch (states.get(currState)) {
                case turn90Clockwise:
                    if (Robot.driveTrain.turnPOM(90, DriveTrain.Direction.CLOCKWISE)) {
                        nextState();
                    }
                    break;
                case drive50Inches:
                    if(Robot.driveTrain.moveGyroDistancePOM(50, DriveTrain.Direction.FORWARD, 1, 0))
                    break;
                default:
                    System.out.println("Auto done");
                    break;
            }
        } else {
            System.out.println("States not initialized");
        }
    }

    public void setStates (ArrayList states) {
        this.states = states;
    }

    public void nextState() {
        currState++;
    }
}
