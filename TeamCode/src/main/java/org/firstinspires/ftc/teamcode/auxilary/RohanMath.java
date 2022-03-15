package org.firstinspires.ftc.teamcode.auxilary;


import org.firstinspires.ftc.teamcode.managers.FeatureManager;

public abstract class RohanMath extends FeatureManager {

    public static int convertDegrees(int degree) {
        if (degree < 0 || degree > 360) {
            return -1;
        } else {
            int newDeg = degree - 180;
        }
        return degree;
    }

}
