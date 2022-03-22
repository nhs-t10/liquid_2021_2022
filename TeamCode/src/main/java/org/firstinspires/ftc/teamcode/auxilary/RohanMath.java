package org.firstinspires.ftc.teamcode.auxilary;


import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.teamcode.managers.FeatureManager;

public abstract class RohanMath extends FeatureManager {

    public static int convertDeg(int degree) {
        if (degree < 0 || degree > 360) {
            return -1;
        } else {
            int newDeg = degree - 180;
        }
        return degree;
    }
    public static double getDistanceTrig(double x, double y) {
         double c = Math.pow((Math.pow(x, 2) + Math.pow(y, 2)), 1/2);
         double alpha = Math.toDegrees(Math.asin(y/c));
         double theta = 90 - alpha;
         double h = x*Math.cos(Math.toRadians(theta));
         return h;

    }


}
