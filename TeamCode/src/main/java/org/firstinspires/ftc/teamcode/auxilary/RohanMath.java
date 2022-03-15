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
    public static double getDistanceTrig(double d1, double d2){
        int theta = 90;
        if(d1 > 300){
            d1 = d2;
        }
        else if(d2 > 300){
            d2 = d1;
        }
        double h = (((d2)/Math.pow(2.0, 0.5))+((d1)/Math.pow(2.0, 0.5)))/2;
        return h;
    }

}
