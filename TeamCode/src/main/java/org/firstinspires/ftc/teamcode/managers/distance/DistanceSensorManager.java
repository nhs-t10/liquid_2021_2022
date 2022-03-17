package org.firstinspires.ftc.teamcode.managers.distance;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.managers.FeatureManager;

public class DistanceSensorManager extends FeatureManager {

    public Rev2mDistanceSensor[] distanceSensors;
    public String[] distanceSensorNames;

    public DistanceSensorManager() {
    }

    public DistanceSensorManager(Rev2mDistanceSensor[] _distanceSensor) {
        this.distanceSensors = _distanceSensor;
    }

    public int checkDistSensor(String distSensorName) {
        for (int i = 0; i < distanceSensorNames.length; i++) {
            String currentName = distanceSensorNames[i];

            if (currentName == distSensorName)
        }

    }
    public void getDistance(String distSensorName) {
        for (int i = 0; i< distanceSensorNames.length; i++){
            String currentName = distanceSensorNames[i];

            if (currentName == distSensorName) {

            }
            else if ()
        }
    }
}
