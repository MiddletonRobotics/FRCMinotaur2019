package frc.robot.Autonomous;

import frc.robot.Autonomous.StateMachine.AutoStates;
import frc.robot.Robot;
import java.util.Arrays;
import java.util.List;

public class TurnTest extends Auto {

    private List<AutoStates> statesList =  Arrays.asList(AutoStates.drive50Inches/*, AutoStates.turn90Clockwise*/);

    public void auto() {
        stateMachine.setStates(statesList);
    }

    public void loop() {
        stateMachine.runStateMachine();
    }

    public void stop() {
        Robot.driveTrain.stopDrive();
    }

}

 