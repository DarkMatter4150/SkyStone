package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.RobotObject;

@Autonomous(name = "Blue Autonomous", group = "Pushbot")
public class BlueAuto extends Auto {

    RobotObject robotObject = new RobotObject(9, 111, 18, 18, 180);

    @Override
    void instructions() {
        resetRobotObject(robotObject);

//        getFoundationBlue();
//        parkBlue();
    }
}
