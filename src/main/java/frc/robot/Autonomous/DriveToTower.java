package frc.robot.Autonomous;

import frc.robot.Autonomous.StateMachine.AutoStates;

import java.util.Arrays;
import java.util.List;

public class DriveToTower extends Auto
{

    private List<AutoStates> statesList =  Arrays.asList(
            AutoStates.drive1foot,
            AutoStates.turn45CounterClockwise,
            AutoStates.drive1foot,
            AutoStates.turn45Clockwise,
            AutoStates.drive1foot,
            AutoStates.limelightApproach,
            AutoStates.limelightApproach,
            AutoStates.limelightApproach
            );

    public void auto() {
        stateMachine.setStates(statesList);
    }

    public void loop() {
        stateMachine.runStateMachine();
        //System.out.println(Robot.driveTrain.getLeftTalon().getSensorCollection().getQuadraturePosition());
    }

    public void stop() {
        //Robot.driveTrain.stopDrive();
    }
}
