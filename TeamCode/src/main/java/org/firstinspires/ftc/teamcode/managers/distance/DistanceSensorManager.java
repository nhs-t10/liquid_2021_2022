package org.firstinspires.ftc.teamcode.managers.distance;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.managers.FeatureManager;

public class DistanceSensorManager extends FeatureManager {

    public Rev2mDistanceSensor[] distanceSensors;
    public String[] distanceSensorNames;


    public DistanceSensorManager(Rev2mDistanceSensor[] _distanceSensor) {
        this.distanceSensors = _distanceSensor;
    }

    public int place = 0;

    public void checkDistSensor(String distSensorName) {
        for (int i = 0; i <= distanceSensorNames.length; i++) {
            String currentName = distanceSensorNames[i];
            if (currentName.equals(distSensorName)) {
                place = i;
            } else if (i == distanceSensorNames.length) {
                throw new IllegalArgumentException("Not a distance sensor on the list");
            }
        }
    }

    public double getDistance(String distSensorName) {
        checkDistSensor(distSensorName);
        return distanceSensors[place].getDistance(CM);

    }
}
