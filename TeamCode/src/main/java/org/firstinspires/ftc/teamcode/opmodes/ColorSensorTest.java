package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Color Sensor Test", group = "Test")
public class ColorSensorTest extends OpMode {

    private RevColorSensorV3 colorSensor;
    private DistanceSensor distanceSensor;

    @Override
    public void init() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
    }

    @Override
    public void loop() {
        RevColorSensorV3 c = colorSensor;
        float r = c.red();
        float g = c.green();
        float b = c.blue();

        double d = distanceSensor.getDistance(DistanceUnit.CM);
        boolean found = r - b <= 30 && d < 6.5f;

        telemetry.addData("COLOR", String.format("%f %f %f", r, g, b));
        telemetry.addData("DISTANCE", String.format("%f", d));
        telemetry.addData("FOUND", found);
        telemetry.update();
    }
}
