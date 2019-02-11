
package frc.robot.Autonomous;
import java.util.ArrayList;

import frc.robot.Autonomous.StateMachine.AutoStates;
import frc.robot.Robot;
import java.util.Arrays;

public class TurnTest extends Auto {

    private ArrayList<AutoStates> statesList = (ArrayList) Arrays.asList(AutoStates.turn90Clockwise, AutoStates.turn90Clockwise);

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

 