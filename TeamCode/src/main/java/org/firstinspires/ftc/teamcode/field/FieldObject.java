package org.firstinspires.ftc.teamcode.field;

public class FieldObject {

    private float[] coordinates;
    private float width;
    private float length;
    private String name;

    public FieldObject(float x, float y, float width, float length, String name) {
        this.coordinates = new float[] {x, y};
        this.width = width;
        this.length = length;
        this.name = name;
    }

    public float[] getCoords() {
        return this.coordinates;
    }

    public float getX() {
        return this.coordinates[0];
    }

    public float getY() {
        return this.coordinates[1];
    }

    public void setX(float x) { this.coordinates[0] = x; }

    public void setY(float y) { this.coordinates[0] = y; }

    public float getWidth() {
        return this.width;
    }

    public float getLength() {
        return this.length;
    }

    public String getName() { return this.name; }
}
