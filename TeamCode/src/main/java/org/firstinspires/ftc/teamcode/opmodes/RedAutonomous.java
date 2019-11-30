package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.breakout.EncoderDrive;
import org.firstinspires.ftc.teamcode.breakout.Robot;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.FieldObject;
import org.firstinspires.ftc.teamcode.field.RobotObject;

@Autonomous(name = "Red Autonomous", group = "Pushbot")
public class RedAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    private Robot robot = new Robot(telemetry);   // Use a Pushbot's hardware
    private RobotObject robotObject;
    private ElapsedTime runtime = new ElapsedTime();
    private Field field = new Field();
    private EncoderDrive encoderDrive;

    @Override
    public void runOpMode() {

        robotObject = new RobotObject(126, 0, 18, 18, "robot", 180);
        encoderDrive = new EncoderDrive(robot, robotObject);

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.setRunMode(Robot.Motor.FRONT_LEFT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(Robot.Motor.FRONT_RIGHT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(Robot.Motor.BACK_LEFT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(Robot.Motor.BACK_RIGHT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.setRunMode(Robot.Motor.FRONT_LEFT, DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setRunMode(Robot.Motor.FRONT_RIGHT, DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setRunMode(Robot.Motor.BACK_LEFT, DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setRunMode(Robot.Motor.BACK_RIGHT, DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d : %7d : %7d : %7d",
                robot.getCurrentPosition(Robot.Motor.FRONT_LEFT),
                robot.getCurrentPosition(Robot.Motor.FRONT_RIGHT),
                robot.getCurrentPosition(Robot.Motor.BACK_LEFT),
                robot.getCurrentPosition(Robot.Motor.BACK_RIGHT));
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //START IN FRONT OF SKYSTONES
        move(field.getObject("r sky stones"), 0, 0, 0, 1000);
//        grabBlock(1000);
        move(field.getObject("mid line"), 12, -20, 0, 0);
        move(field.getObject("mid line"), 12, 20, 0, 1000);
//        dropBlock(1000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    private void move(FieldObject target, float xOffset, float yOffset, int rotationDegrees, long waitMilliseconds) {
        if (opModeIsActive()) {
            EncoderDrive.Data driveMatrix = encoderDrive.getDriveMatrix(target, xOffset, yOffset, rotationDegrees);
            int[] targets = encoderDrive.drive(driveMatrix.fin);
            while (opModeIsActive() &&
                    (robot.isBusy(Robot.Motor.FRONT_LEFT) && robot.isBusy(Robot.Motor.FRONT_RIGHT) &&
                            robot.isBusy(Robot.Motor.BACK_LEFT) && robot.isBusy(Robot.Motor.BACK_RIGHT))) {
                encoderDrive.tick(telemetry, targets, driveMatrix.fin, driveMatrix.teledata);
            }
            encoderDrive.stop();
            sleep(waitMilliseconds);
        }
    }

    private void grabBlock(long workTime) {
        encoderDrive.pickUpBlock();
        pause(workTime);
        encoderDrive.revert("grab");
        pause(workTime);
        encoderDrive.stop();
    }

    private void dropBlock(long workTime) {
        encoderDrive.dropBlock();
        pause(workTime);
        encoderDrive.revert("drop");
        pause(workTime);
        encoderDrive.stop();
    }

    private void pause(long workTime) {
        time = runtime.milliseconds();
        while (opModeIsActive()) {
            if (runtime.milliseconds() - time > workTime) {
                break;
            }
        }
    }
}
