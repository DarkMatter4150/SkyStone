package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.RobotObject;

//@Autonomous(name = "Red Side PARK ONLY", group = "Pushbot")
public class RedAutoPark extends Auto {

    private RobotObject robotObject = new RobotObject(135, 39, 18, 18, 0);

    @Override
    void instructions() {
        resetRobotObject(robotObject);

        parkRed();
    }
}
