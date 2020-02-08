package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Instruction Auto", group="Pushbot")
public class RedInstructionAuto extends Auto {

    @Override
    void instructions() {
        readInstructionFile("skystonered.field");
        skyStonesRed();
        skyStones2Red();
        parkRed();
    }
}
