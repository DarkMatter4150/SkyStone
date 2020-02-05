package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Instruction TXT Test", group="Pushbot")
public class AutoInstructionTest extends Auto {

    @Override
    void instructions() {
        readInstructionFile("output.txt");
    }
}
