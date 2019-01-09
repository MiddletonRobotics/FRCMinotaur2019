// package frc.robot.commands;

// import com.ctre.phoenix.motion.SetValueMotionProfile;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import frc.robot.Robot;
// import frc.robot.Utils;
// import frc.robot.motion.*;

// public class TestProfile extends Auto {

//     MotionProfileRunner leftRunner;
//     MotionProfileRunner rightRunner;

//     private boolean hasSetup;
//     private boolean hasAlreadyRan;

//     @Override
//     public void auto() {
//         try {
//             Utils.resetRobot();
//             leftRunner = new MotionProfileRunner(Robot.driveTrain.getLeftTalon(), new LeftTalonTestMProfile(), false);
//             rightRunner = new MotionProfileRunner(Robot.driveTrain.getRightTalon(), new RightTalonTestMProfile(),true);
//             hasSetup = false;
//             hasAlreadyRan = false;

//             // Robot.driveTrain.moveByGyroDistance(130 , DriveTrain.Direction.BACKWARD, 0.5, 2, 5000);
//             //Robot.driveTrain.driveVelocity(800, -800);
//             rightRunner.startMotionProfile();
//             leftRunner.startMotionProfile();
//             leftRunner.control();
//             rightRunner.control();
//             hasSetup = true;
//             // Robot.driveTrain.stopDrive();
//         } catch (Exception e) {
//             e.printStackTrace();
//             Robot.driveTrain.stopDrive();
//         }

//     }

//     @Override
//     public void loop() {
//         leftRunner.control();
//         rightRunner.control();
//         Robot.driveTrain.getLeftTalon().set(ControlMode.MotionProfile, leftRunner.getSetValue().value);
//         Robot.driveTrain.getRightTalon().set(ControlMode.MotionProfile, rightRunner.getSetValue().value);
//         if (hasSetup && !hasAlreadyRan && leftRunner.getSetValue().equals(SetValueMotionProfile.Hold) && rightRunner.getSetValue().equals(SetValueMotionProfile.Hold)) {
//             hasAlreadyRan = true;
//             System.out.println("mhs tigers please listen up because you will need this. GRAB PEN PENCIL AND COPY THIS");
//         }
//     }

//     @Override
//     public void stop() {
//         Robot.driveTrain.getLeftTalon().set(ControlMode.MotionProfile, 0);
//         Robot.driveTrain.getRightTalon().set(ControlMode.MotionProfile,0);
//         Robot.driveTrain.stopDrive();
//         leftRunner.reset();
//         rightRunner.reset();
//        // leftRunner = null;
//        // rightRunner = null;
//     }
// }
