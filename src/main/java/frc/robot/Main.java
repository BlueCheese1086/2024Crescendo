// DO NOT EDIT THIS CLASS
// ALL MANIPULATION TO THE ROBOT AND ITS SYSTEMS MUST BE DONE ELSEWHERE!!!
package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}