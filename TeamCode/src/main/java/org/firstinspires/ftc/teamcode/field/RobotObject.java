package org.firstinspires.ftc.teamcode.field;

public class RobotObject extends FieldObject {

    private float rotation;
    public final static float RADIUS = 25.456f;

    public RobotObject(float x, float y, float width, float length, String name, float rotation) {
        super(x, y, width, length, name);
        this.rotation = rotation;
    }

    public float getRotation() {
        return this.rotation;
    }

    public void setRotation(float degrees) {
        this.rotation = degrees;
    }

    public void addRotation(float degrees) {
        this.rotation += degrees;
    }
}
