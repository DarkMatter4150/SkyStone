//package ExampleCode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Auto;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//
//import java.io.File;
//import java.util.ArrayList;
//import java.util.List;
//
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
//import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
//import org.firstinspires.ftc.teamcode.BreakoutMotor;
//import org.firstinspires.ftc.teamcode.breakout.BreakoutREVGyro;
//import org.firstinspires.ftc.teamcode.breakout.BreakoutServo;
//
//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
//import static org.firstinspires.ftc.teamcode.BreakoutMotor.Direction.MOTOR_R;
//import static org.firstinspires.ftc.teamcode.BreakoutMotor.Direction.MOTOR_F;
//
///**
// * This class is used for the Auto segment of the game. No controllers are to be used.
// **/
//@Auto(name = "Auto", group = "pushbot")
//
//public class BreakoutAutoOp extends OpMode {
//
//    private int goldLoc = -1;
//    private boolean moveBotOnce = true;
//    private boolean resetTimer = true;
//    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
//    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
//    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
//    private static final String VUFORIA_KEY = "AT5/naD/////AAABmaUvi2GyUUirkbnkPyo71cQmkhAaUVAKxkF96tVK5PueT2cxswHD7dT7ZBvwEBS4oPEDsajO7QdUFF0VtshBOnfi5AFTlL5pfgmMQhr2ceo9xTa+pyw71Bvym3qvmv5utemN+yU5b8eQfejOyfCh/EGpSvclNB9JAa5125hevdhFoYOT7BU6/Xx1wBAith2m/RPesA1tqqBfjWC4KfBonwCtLa0+erPLNnGt0PJiM0CIxyFTo4u9V4AP1TQm/VooDJ8+yXN7ZiIimPG42+0ahoYOdBK6Tc3jX2idV4YCSPBv8Gz6Ut9fu9VLMiYkDUeGZ9prKeehGh7NtttajEe6Dw/h3ww1sCoKOTfJvK0dWi3h";
//    private int captureCounter = 0;
//    private File captureDirectory = AppUtil.ROBOT_DATA_DIR;
//    private TFObjectDetector tfod;
//    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
//    //    private WebcamName webcamName;
//    private boolean found = false;
//    private boolean firstRun = true;
//    private boolean run = true;
//    private String trackableName = "";
//    double i = 0;
//    private int goldMineralX = -1;
//    private int silverMineral1X = -1;
//    private int silverMineral2X = -1;
//
//    private List<VuforiaTrackable> allTrackables = new ArrayList<>();
//    private List<VuforiaTrackable> allTrackables2 = new ArrayList<>();
//    private List<BreakoutMotor> allMotors = new ArrayList<>();
//    private List<BreakoutServo> allServos = new ArrayList<>();
//
//    private BreakoutMotor motorLeft = new BreakoutMotor();
//    private BreakoutMotor motorRight = new BreakoutMotor();
//    private BreakoutMotor motorSweeper = new BreakoutMotor();
//    private BreakoutMotor motorSweeperArm = new BreakoutMotor();
//    private BreakoutMotor motorHorizontal = new BreakoutMotor();
//    private BreakoutMotor motorVertical = new BreakoutMotor();
//
//    private BreakoutREVGyro gyroA = new BreakoutREVGyro();
//
//    private ElapsedTime startTime = new ElapsedTime();
//
//    private double integral;
//    private double last_error;
//
//    private void allSet(List<BreakoutMotor> motors, int job) {
//        for (BreakoutMotor motor : motors) {
//            setThing(motor, job);
//        }
//    }
//
//    private void setThing(BreakoutMotor motor, int job) {
//        if (job == 1) {
//            motor.setPower(0);
//        } else if (job == 2) {
//            motor.setDirection(MOTOR_F);
//        }
//    }
//
////    double Kp = 0.5;
////    double Ki = 0.0;
////    double Kd = 0.0;
////
////    private double pid(double cenLinAng) {
////        double delta_time = startTime.now(TimeUnit.MILLISECONDS);
////        double error = cenLinAng - gyroA.getOrient().firstAngle;
////        integral += error * delta_time;
////        double derivative = (error - last_error) / delta_time;
////        last_error = error;
////        double var = (error * Kp) + (integral * Ki) + (derivative * Kd);
////        double out = 1-var;
////        telemetry.addData("angle", gyroA.getOrient().firstAngle);
////        telemetry.addData("delta_time", delta_time);
////        telemetry.addData("error", error);
////        telemetry.addData("accumulation of error", integral);
////        telemetry.addData("derivative of error", derivative);
////        telemetry.addData("last error", last_error);
////        telemetry.addData("out", out);
////        return out;
////    }
////    private double pid2(double cenLinAng) {
////        double delta_time = startTime.now(TimeUnit.MILLISECONDS);
////        double error = cenLinAng + gyroA.getOrient().firstAngle;
////        integral += error * delta_time;
////        double derivative = (error - last_error) / delta_time;
////        last_error = error;
////        double var = (error * Kp) + (integral * Ki) + (derivative * Kd);
////        double out = 1-var;
////        telemetry.addData("angle", gyroA.getOrient().firstAngle);
////        telemetry.addData("delta_time", delta_time);
////        telemetry.addData("error", error);
////        telemetry.addData("accumulation of error", integral);
////        telemetry.addData("derivative of error", derivative);
////        telemetry.addData("last error", last_error);
////        telemetry.addData("out", out);
////        return out;
////    }
////
////    private void blueRover() {
////        double centerAng = 0;
////        if(gyroA.getOrient().firstAngle <= centerAng) {
////            motorLeft.setPower(Math.max(0, Math.min(1, pid(centerAng)))+0.5);
////            motorRight.setPower(0.5);
////        } else {
////            motorLeft.setPower(0.5);
////            motorRight.setPower(Math.max(0, Math.min(1, pid2(centerAng)))+0.5);
////        }
////        telemetry.update();
////    }
//
//    private void blueRover(int goldLoc) {
//        if (resetTimer) {
//            startTime.reset();
//            resetTimer = false;
//        }
//        while (startTime.milliseconds() < 1000) {
//            motorVertical.setPower(-1);
//        }
//        motorVertical.setPower(0);
//        if (goldLoc != -1 && moveBotOnce) {
//            double time2 = startTime.milliseconds();
//            while (startTime.milliseconds() < time2+1500) {
//                if (goldLoc == 0) {
//                    motorLeft.setPower(.5);
//                    motorRight.setPower(1);
//                    telemetry.addData("left", null);
//                } else if (goldLoc == 1) {
//                    motorLeft.setPower(1);
//                    motorRight.setPower(1);
//                    telemetry.addData("middle", null);
//                } else if (goldLoc == 2) {
//                    motorLeft.setPower(1);
//                    motorRight.setPower(.5);
//                    telemetry.addData("right", null);
//                }
//            }
//            moveBotOnce = false;
//        }
//        if (startTime.milliseconds() >= 15000 && goldLoc == -1) {
//            if (moveBotOnce) {
//                double time1 = startTime.milliseconds();
//                while (startTime.milliseconds() < 1000+time1) {
//                    motorLeft.setPower(1);
//                    motorRight.setPower(1);
//                }
//                while (startTime.milliseconds() < 1500+time1) {
//                    motorLeft.setPower(0);
//                    motorRight.setPower(0);
//                    motorSweeperArm.setPower(1);
//                }
//                while (startTime.milliseconds() < 2000+time1) {
//                    motorSweeperArm.setPower(0);
//                    motorSweeper.setPower(-0.5);
//                }
//                moveBotOnce = false;
//            }
//            allSet(allMotors, 1);
//        }
//    }
//
//    private void redFootprint(int goldLoc) {
//
//    }
//
//    private void frontCraters(int goldLoc) {
//
//    }
//
//    private void backSpace(int goldLoc) {
//
//    }
//
//    private void initTfod(VuforiaLocalizer vuforia) {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
//    }
//
//    @Override
//    public void init() {
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
////        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        VuforiaLocalizer.Parameters tfParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//
////        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        tfParameters.cameraDirection = CAMERA_CHOICE;
//        tfParameters.vuforiaLicenseKey = VUFORIA_KEY;
////        parameters.cameraName = webcamName;
//
//        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);
////        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
//        VuforiaLocalizer vuforiaTensor = ClassFactory.getInstance().createVuforia(tfParameters);
//        vuforiaTensor.enableConvertFrameToBitmap();
////        VuforiaTrackables targetsRoverRuckus = vuforia.loadTrackablesFromAsset("RoverRuckus");
////        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
////        blueRover.setName("Blue-Rover");
////        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
////        redFootprint.setName("Red-Footprint");
////        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
////        frontCraters.setName("Front-Craters");
////        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
////        backSpace.setName("Back-Space");
//
////        allTrackables.addAll(targetsRoverRuckus);
//
////        targetsRoverRuckus.activate();
//        startTime.reset();
//
//        motorLeft.set(hardwareMap.dcMotor.get("motorLeft"));
//        motorRight.set(hardwareMap.dcMotor.get("motorRight"));
//        motorSweeper.set(hardwareMap.dcMotor.get("motorSweeper"));
//        motorSweeperArm.set(hardwareMap.dcMotor.get("motorSweeperArm"));
//        motorHorizontal.set(hardwareMap.dcMotor.get("motorHorizontal"));
//        motorVertical.set(hardwareMap.dcMotor.get("motorVertical"));
//        motorLeft.setDirection(MOTOR_R);
//        motorRight.setDirection(MOTOR_R);
//        motorSweeper.setDirection(MOTOR_F);
//        motorSweeperArm.setDirection(MOTOR_F);
//        motorHorizontal.setDirection(MOTOR_F);
//        allMotors.add(motorLeft);
//        allMotors.add(motorRight);
//        allMotors.add(motorSweeper);
//        allMotors.add(motorSweeperArm);
//        allMotors.add(motorHorizontal);
//        allSet(allMotors, 1);
//
//        //webcamName = hardwareMap.get(WebcamName.class, "Webcam");
//
//        //Broken out Gyro class
//        gyroA.set(hardwareMap.get(gyroA.IMU, "gyroA"));
//        telemetry.addLine("Calibrating: DO NOT MOVE!");
//        telemetry.update();
//        gyroA.calibrate();
//        telemetry.clearAll();
//        telemetry.update();
//
//        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
//            initTfod(vuforiaTensor);
//        } else {
//            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
//        }
//
//        if (tfod != null) {
//            tfod.activate();
//        }
//        startTime.reset();
//    }
//
//    @Override
//    public void start() {
//        allSet(allMotors, 1);
//        startTime.reset();
//    }
//
//
//    @Override
//    public void loop() {
//
////        if (!found) {
////            while (startTime.milliseconds() < 100) {
////                motorLeft.setPower(1);
////                motorRight.setPower(1);
////            }
////            motorLeft.setPower(-0.2);
////            motorRight.setPower(0.2);
////            for (VuforiaTrackable trackable : allTrackables) {
////                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
////                    this.trackableName = trackable.getName();
////                    telemetry.addData("Visible Target", trackable.getName());
////                    found = true;
////                    startTime.reset();
////                    break;
////                }
////            }
////        }
//
//        if (tfod != null) {
//            // getUpdatedRecognitions() will return null if no new information is available since
//            // the last time that call was made.
//            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//            if (updatedRecognitions != null) {
//                telemetry.addData("# Object Detected", updatedRecognitions.size());
//                if (updatedRecognitions.size() == 3) {
//                    for (Recognition recognition : updatedRecognitions) {
//                        telemetry.addData("label",recognition.getLabel());
//                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
//                            goldMineralX = (int) recognition.getLeft();
//                        } else if (silverMineral1X == -1) {
//                            silverMineral1X = (int) recognition.getLeft();
//                        } else {
//                            silverMineral2X = (int) recognition.getLeft();
//                        }
//                    }
//
//                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
//                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
//                            telemetry.addData("Gold Mineral Position", "Left");
//                            goldLoc = 0;
//                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
//                            telemetry.addData("Gold Mineral Position", "Right");
//                            goldLoc = 2;
//                        } else {
//                            telemetry.addData("Gold Mineral Position", "Center");
//                            goldLoc = 1;
//                        }
//                    }
//                }
//            }
//        }
//        blueRover(goldLoc);
//        telemetry.addData("gold",goldMineralX);
//        telemetry.addData("silver1",silverMineral1X);
//        telemetry.addData("silver2",silverMineral2X);
//        telemetry.addData("time", startTime.milliseconds());
//        telemetry.update();
////        if(run) {
////            switch (trackableName) {
////                case "Blue-Rover":
////                    startTime.reset();
////                    blueRover(goldLoc);
////                    run = false;
////                    break;
////                case "Red-Footprint":
////                    startTime.reset();
////                    redFootprint(goldLoc);
////                    run = false;
////                    break;
////                case "Front-Craters":
////                    startTime.reset();
////                    frontCraters(goldLoc);
////                    run = false;
////                    break;
////                case "Back-Space":
////                    startTime.reset();
////                    backSpace(goldLoc);
////                    run = false;
////                    break;
////                case "":
////                    break;
////            }
////        }
//    }
//
//    @Override
//    public void stop() {
//        allSet(allMotors, 1);
//    }
//}
