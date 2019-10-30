package org.firstinspires.ftc.teamcode.field;

public class BuildSite extends FieldObject {

    private float[] coordinates;
    private float width;
    private float length;
    private boolean redSite;

    public BuildSite(float x, float y, float width, float length, String name, boolean redSite) {
        super(x, y, width, length, name);
        this.coordinates = new float[] {x, y};
        this.width = width;
        this.length = length;
        this.redSite = redSite;
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

    public float getWidth() {
        return this.width;
    }

    public float getLength() {
        return this.length;
    }

    public boolean isRedSite() {
        return this.redSite;
    }
}
