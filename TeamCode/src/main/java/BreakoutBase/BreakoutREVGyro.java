package BreakoutBase;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This class is used to simplify the usage of Gyroscopes in the code.
 * Please use this class instead of the BNO055IMU class.
 **/
public class BreakoutREVGyro {

    //Define objects
    public Class<BNO055IMU> IMU = BNO055IMU.class;
    private BNO055IMU imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    //State for telemetry
    Orientation angles;
    Acceleration gravity;

    public BNO055IMU get() {
        return imu;
    }

    public void set(BNO055IMU gyro) {
        this.imu = gyro;
    }

    void calibrate() {
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
    }

    public boolean isCalibrating() {
        return !imu.isGyroCalibrated();
    }

    public Acceleration getAccel() {
        return imu.getAcceleration();
    }

    public Orientation getOrient() {
        return imu.getAngularOrientation();
    }

    public Velocity getVel() {
        return imu.getVelocity();
    }

    public Position getPos() {
        return imu.getPosition();
    }

}
