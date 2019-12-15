package org.firstinspires.ftc.teamcode.field;

public class RobotObject extends FieldObject {

    private float rotation;

    /**
     * A field object modified to be used for the robot.
     *
     * @param xCenter Center of the robot x position.
     * @param yCenter Center of the robot y position.
     * @param width Width of the robot (X distance, usually 18).
     * @param length Length of the robot (Y distance, usually 18).
     * @param rotation Direction the robot is facing, as though the robot was the center of a unit circle. (Degrees)
     */
    public RobotObject(float xCenter, float yCenter, float width, float length, float rotation) {
        // xCenter - (width/2) to turn center x coordinate into the bottom left coordinate.
        // yCenter - (length/2) to turn center y coordinate into the bottom left coordinate.
        super(xCenter - (width/2), yCenter - (length/2), width, length, "robot");
        this.rotation = rotation;
    }

    /**
     * Method to get the rotation of the robot (Direction it is facing in degrees).
     *
     * @return Returns rotation in degrees.
     */
    public float getRotation() {
        return this.rotation;
    }

    /**
     * Method to set the rotation of the robot (Direction it is facing in degrees).
     *
     * @param degrees New angle to set the rotation to.
     */
    @Deprecated
    public void setRotation(float degrees) {
        this.rotation = degrees;
    }

    /**
     * Method to add a certain amount of degrees to the robot's rotation.
     *
     * @param degrees Angle in degrees to add to the robot's rotation.
     */
    public void addRotation(float degrees) {
        this.rotation += degrees;
    }
}
