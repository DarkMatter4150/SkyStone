package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.RobotObject;

@Autonomous(name = "Red Autonomous", group = "Pushbot")
public class RedAuto extends Auto {

    RobotObject robotObject = new RobotObject(135, 111, 18, 18, 0);

    @Override
    void instructions() {
        resetRobotObject(robotObject);

//        rotate();
        getFoundationRed();
//        parkRed();
    }
}
