package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Instruction Auto", group="Pushbot")
public class BlueInstructionAuto extends Auto {

    @Override
    void instructions() {
        //Hi
        readInstructionFile("blue2.field");
    }
}
