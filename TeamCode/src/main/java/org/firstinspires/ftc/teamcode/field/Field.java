package org.firstinspires.ftc.teamcode.field;

import java.util.HashMap;
import java.util.Map;

/**
 * Field class containing a HashMap of all of the different field objects on the field. Use this to build the field for use in autonomous.
 */
public class Field {

    private Map<String, FieldObject> fieldObjects;

    /* Constructor */
    public Field() {
        this.fieldObjects = new HashMap<>();

        FieldObject redDepot = new FieldObject(0.0f, 0.0f, 24.0f, 24.0f, "r depot");
        FieldObject blueDepot = new FieldObject(10.0f, 0.0f, 24.0f, 24.0f, "b depot");
        FieldObject redSkyStones = new FieldObject(92.0f, 0.0f, 4.0f, 48.0f, "r sky stones");
        FieldObject blueSkyStones = new FieldObject(48.0f, 0.0f, 4.0f, 48.0f, "b sky stones");
        FieldObject redFoundation = new FieldObject(96.75f, 140.0f, -18.5f, -34.5f, "r foundation");
        FieldObject blueFoundation = new FieldObject(47.25f, 140.0f, 18.5f, -34.5f, "b foundation");
        BuildSite redBuildSite = new BuildSite(120.0f, 120.0f, 24.0f, 24.0f, "r build site", true);
        BuildSite blueBuildSite = new BuildSite(0.0f, 120.0f, 24.0f, 24.0f, "b build site", false);
        FieldObject buildZone = new FieldObject(0.0f, 7.0f, 144.0f, 60.0f, "build zone");
        FieldObject loadingZone = new FieldObject(0.0f, 0.0f, 144.0f, 60.0f, "loading zone");
        FieldObject redBridge = new FieldObject(96.0f, 69.6f, 48.0f, 4.8f, "r bridge");
        FieldObject blueBridge = new FieldObject(0.0f, 69.6f, 48.0f, 4.8f, "b bridge");
        FieldObject neutralBridge = new FieldObject(48.0f, 67.2f, 48.0f, 9.6f, "n bridge");
        FieldObject midLine = new FieldObject(0, 72, 144, 0, "mid line");
        fieldObjects.put(redDepot.getName(), redDepot);
        fieldObjects.put(blueDepot.getName(), blueDepot);
        fieldObjects.put(redSkyStones.getName(), redSkyStones);
        fieldObjects.put(blueSkyStones.getName(), blueSkyStones);
        fieldObjects.put(redFoundation.getName(), redFoundation);
        fieldObjects.put(blueFoundation.getName(), blueFoundation);
        fieldObjects.put(redBuildSite.getName(), redBuildSite);
        fieldObjects.put(blueBuildSite.getName(), blueBuildSite);
        fieldObjects.put(buildZone.getName(), buildZone);
        fieldObjects.put(loadingZone.getName(), loadingZone);
        fieldObjects.put(redBridge.getName(), redBridge);
        fieldObjects.put(blueBridge.getName(), blueBridge);
        fieldObjects.put(neutralBridge.getName(), neutralBridge);
        fieldObjects.put(midLine.getName(), midLine);
    }

    /**
     * Adds an object to the HashMap.
     *
     * @param name Name of the object.
     * @param object {@link FieldObject} to add to the HashMap.
     */
    public void addObject(String name, FieldObject object) {
        fieldObjects.put(name, object);
    }

    /**
     * Removes an object from the map using the name.
     *
     * @param name Name of the object to remove.
     */
    public void removeObject(String name) {
        fieldObjects.remove(name);
    }

    /**
     * Gets an object from the HashMap.
     *
     * @param name Name of the object to return.
     * @return Returns the {@link FieldObject} with the given name.
     */
    public FieldObject getObject(String name) {
        return fieldObjects.get(name);
    }
}
