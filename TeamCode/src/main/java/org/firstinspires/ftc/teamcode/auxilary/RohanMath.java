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
    public static double doubleAve(double[] array){
        double sum = 0;
        for(int i = 0; i < array.length; i++){
            sum = sum + array[i];
        }
        return sum/array.length;
    }
    public static double intAve(int[] array){
        double sum = 0;
        for(int i = 0; i < array.length; i++){
            sum = sum + array[i];
        }
        return sum/array.length;
    }

    public static double[] getInputOmniValues(double x, double y) {
        double maxValue = Math.max(Math.abs(y) + Math.abs(x), 1);
        double flPower = (y + x)/maxValue;
        double blPower = (y - x) / maxValue;
        double frPower = (y - x) / maxValue;
        double brPower = (y + x) / maxValue;

        double[] omniDriveValues = {flPower,frPower,blPower,brPower};

        return omniDriveValues;
    }

}
