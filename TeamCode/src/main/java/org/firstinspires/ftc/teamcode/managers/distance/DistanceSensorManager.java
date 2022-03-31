package org.firstinspires.ftc.teamcode.managers.distance;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auxilary.RohanMath;
import org.firstinspires.ftc.teamcode.managers.FeatureManager;

import java.lang.reflect.Array;
import java.util.Arrays;

public class DistanceSensorManager extends FeatureManager {

    public Rev2mDistanceSensor[] distanceSensors;
    public String[] distanceSensorNames;



    public ElapsedTime timer = new ElapsedTime();

    public DistanceSensorManager(Rev2mDistanceSensor[] _distanceSensor) {
        this.distanceSensors = _distanceSensor;
    }

    public int place = 0;

    public void delay (double milliseconds){
        while (timer.milliseconds() < milliseconds){
            //do nothing
        }
    }

    public void checkDistSensor(String distSensorName) {
        boolean isIn = false;
        for (int i = 0; i < distanceSensorNames.length; i++) {
            String currentName = distanceSensorNames[i];
            if (currentName.equals(distSensorName)) {
                isIn = true;
                place = i;
            }
        }
        if (!isIn) {
            throw new IllegalArgumentException("Distance sensor "+distSensorName+" not in array");
        }
    }
    public double getDistance(String distSensorName) {
        checkDistSensor(distSensorName);
        return distanceSensors[place].getDistance(CM);

    }

    public double getDistanceTrig(String distSensorName1, String distSensorName2) {
        checkDistSensor(distSensorName1);
        double dist1 = distanceSensors[place].getDistance(CM);
        checkDistSensor(distSensorName2);
        double dist2 = distanceSensors[place].getDistance(CM);
        return RohanMath.getDistanceTrig(dist1, dist2);
    }
    public double getDistanceAverage2(String distSensorName1, String distSensorName2){
        checkDistSensor(distSensorName1);
        double dist1 = distanceSensors[place].getDistance(CM);
        checkDistSensor(distSensorName2);
        double dist2 = distanceSensors[place].getDistance(CM);
        double[] distSensors = {dist1,dist2};
        return RohanMath.doubleAve(distSensors);
    }
    public double getDistanceAverage3(String distSensorName1, String distSensorName2, String distSensorName3){
        checkDistSensor(distSensorName1);
        double dist1 = distanceSensors[place].getDistance(CM);
        checkDistSensor(distSensorName2);
        double dist2 = distanceSensors[place].getDistance(CM);
        checkDistSensor(distSensorName3);
        double dist3 = distanceSensors[place].getDistance(CM);
        double[] distSensors = {dist1,dist2,dist3};
        return RohanMath.doubleAve(distSensors);
    }
    public double getDistanceAverage4(String distSensorName1, String distSensorName2, String distSensorName3, String distSensorName4){
        checkDistSensor(distSensorName1);
        double dist1 = distanceSensors[place].getDistance(CM);
        checkDistSensor(distSensorName2);
        double dist2 = distanceSensors[place].getDistance(CM);
        checkDistSensor(distSensorName3);
        double dist3 = distanceSensors[place].getDistance(CM);
        checkDistSensor(distSensorName4);
        double dist4 = distanceSensors[place].getDistance(CM);
        double[] distSensors = {dist1,dist2,dist3,dist4};
        return RohanMath.doubleAve(distSensors);
    }
public double  waitforDistChange(String distSensorName, double checkTimeMilliseconds){
    checkDistSensor(distSensorName);
    double currentDist = distanceSensors[place].getDistance(CM);
        for(int i = 0; i < 1;){
            if (currentDist != distanceSensors[place].getDistance(CM)){
                i++;
            }
            else{
                currentDist = distanceSensors[place].getDistance(CM);
                delay(checkTimeMilliseconds);
            }
        }
    return distanceSensors[place].getDistance(CM);
}

}