package ExampleCode;

import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.List;

/**
 * This class is used to simplify the usage of Gyroscopes in the code.
 * Please use this class instead of the IntegratingGyroscope or GyroSensor classes.
 **/
@Deprecated
public class BreakoutGyro {

    //Define objects
    private IntegratingGyroscope gyro;
    GyroSensor gyroSensor;

    //Create arrays for raw xyz, angle orientation xyz, and angular velocity xyz data
    private List<Integer> raw = new ArrayList<>();
    private List<Float> angle = new ArrayList<>();
    private List<Float> velocity = new ArrayList<>();

    //Calibrate the sensor
    void calibrate() {
        gyroSensor.calibrate();
    }

    //Retrieve gyroSensor var
    public GyroSensor get() {
        return gyroSensor;
    }

    //Set hardware map data of gyro
    public void set(GyroSensor gyro) {
        this.gyroSensor = gyro;
    }

    //Retrieve the raw xyz data array
    List<Integer> getRawXYZ() {
        raw.add(gyroSensor.rawX());
        raw.add(gyroSensor.rawY());
        raw.add(gyroSensor.rawZ());
        return raw;
    }

    //Retrieve the angular orientation xyz data array
    List<Float> getOrientationXYZ() {
        gyro = (IntegratingGyroscope) gyroSensor;
        Orientation angularOrientation = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        angle.add(angularOrientation.firstAngle);
        angle.add(angularOrientation.secondAngle);
        angle.add(angularOrientation.thirdAngle);
        return angle;
    }

    //Retrieve the angular velocity xyz data array
    List<Float> getAngleVelocityXYZ() {
        gyro = (IntegratingGyroscope) gyroSensor;
        AngularVelocity angularVelocity = gyro.getAngularVelocity(AngleUnit.DEGREES);
        velocity.add(angularVelocity.xRotationRate);
        velocity.add(angularVelocity.yRotationRate);
        velocity.add(angularVelocity.zRotationRate);
        return velocity;
    }

    //Retrieve the heading of the sensor
    int getHeading() {
        return gyroSensor.getHeading();
    }

}
