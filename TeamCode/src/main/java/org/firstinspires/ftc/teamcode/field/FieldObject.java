package org.firstinspires.ftc.teamcode.field;

public class FieldObject {

    private float[] coordinates;
    private float width;
    private float length;
    private String name;

    /**
     * Constructor to create a field object.
     *
     * @param x Bottom left corner x position of the object (in inches).
     * @param y Bottom left corner y position of the object (in inches).
     * @param width Width of the field object. (X distance)
     * @param length Length of the field object. (Y distance)
     * @param name Name of the field object used to figure out which one it is.
     */
    public FieldObject(float x, float y, float width, float length, String name) {
        this.coordinates = new float[] {x, y};
        this.width = width;
        this.length = length;
        this.name = name;
    }

    /**
     * Method to get the X position of the field object (Bottom left corner x).
     *
     * @return Returns X position.
     */
    public float getX() {
        return this.coordinates[0];
    }

    /**
     * Method to get the Y position of the field object (Bottom left corner y).
     *
     * @return Returns Y position.
     */
    public float getY() {
        return this.coordinates[1];
    }

    /**
     * Method to set the X position of the field object (Bottom left corner x).
     *
     * @param x Float to set the x coordinate of the field object to.
     */
    public void setX(float x) { this.coordinates[0] = x; }

    /**
     * Method to set the Y position of the field object (Bottom left corner y).
     *
     * @param y Float to set the y coordinate of the field object to.
     */
    public void setY(float y) { this.coordinates[1] = y; }

    /**
     * Method to get the width of the field object.
     *
     * @return Returns width of the object (X distance).
     */
    public float getWidth() {
        return this.width;
    }

    /**
     * Method to get the length of the field object.
     *
     * @return Returns length of the object (Y distance).
     */
    public float getLength() {
        return this.length;
    }

    /**
     * Method to get the name of the field object.
     *
     * @return Returns name of the object.
     */
    public String getName() { return this.name; }
}
