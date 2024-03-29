package org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model;

import com.google.firebase.database.annotations.NotNull;

import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.values.AutoautoNumericValue;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.model.values.AutoautoString;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.AutoautoRuntimeVariableScope;
import org.firstinspires.ftc.teamcode.auxilary.dsls.autoauto.runtime.AutoautoSystemVariableNames;

import java.util.HashMap;
import java.util.Map;

public class AutoautoProgram implements AutoautoProgramElement {
    public static String fileName;
    private String initialPathName;
    private String oldPathName;
    public HashMap<String, Statepath> paths;
    public Statepath currentPath;

    AutoautoRuntimeVariableScope scope;
    Location location;


    public static String formatStack(Location location) {
        return "\n\tat " + fileName + ":" + location.line + ":" + location.col + "\n\tat " + location.statepath + ", state " + location.stateNumber;
    }

    public static AutoautoProgram P(HashMap<String, Statepath> s, String initialPathName) {
        return new AutoautoProgram(s, initialPathName);
    }

    public AutoautoProgram(HashMap<String, Statepath> s, String initialPathName) {
        this.paths = s;
        this.initialPathName = initialPathName;
        this.currentPath = paths.get(initialPathName);
    }
    public void loop() {
        String currentStatepathName = ((AutoautoString)(scope.get(AutoautoSystemVariableNames.STATEPATH_NAME))).getString();

        //if steps have changed, init the new one
        if(!currentStatepathName.equals(this.oldPathName)) {
            //reset current state
            scope.systemSet(AutoautoSystemVariableNames.STATE_NUMBER, new AutoautoNumericValue(0));

            this.currentPath = this.paths.get(currentStatepathName);
            this.currentPath.stepInit();
            this.oldPathName = currentStatepathName;
        }

        this.currentPath.loop();
    }

    public void init() {
        for(Statepath p : this.paths.values()) p.init();
        scope.systemSet(AutoautoSystemVariableNames.STATEPATH_NAME, new AutoautoString(this.initialPathName));
        scope.systemSet(AutoautoSystemVariableNames.STATE_NUMBER, new AutoautoNumericValue(0));
    }

    public void stepInit() {

        for(Statepath p : this.paths.values()) p.stepInit();
    }

    @NotNull
    public String toString() {
        StringBuilder pathsStr = new StringBuilder();
        for(Map.Entry<String, Statepath> entry : paths.entrySet()) {
            pathsStr.append(entry.getKey()).append(": ").append(entry.getValue().toString()).append("\n.\n");
        }
        return pathsStr.toString();
    }

    @Override
    public AutoautoRuntimeVariableScope getScope() {
        return scope;
    }

    @Override
    public void setScope(AutoautoRuntimeVariableScope scope) {
        this.scope = scope;
        for(Statepath s : paths.values()) s.setScope(scope);
    }

    @Override
    public Location getLocation() {
        return location;
    }

    @Override
    public void setLocation(Location location) {
        this.location = location;
    }

    @Override
    public AutoautoProgram clone() {
        HashMap<String, Statepath> pathsCloned = new HashMap<>();
        for(String p : paths.keySet()) {
            pathsCloned.put(p, paths.get(p).clone());
        }
        AutoautoProgram c = new AutoautoProgram(pathsCloned, initialPathName);
        c.setLocation(location);
        return c;
    }
}
