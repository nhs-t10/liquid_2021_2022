package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.managers.FeatureManager;
import org.firstinspires.ftc.teamcode.managers.imu.ImuManager;
import org.firstinspires.ftc.teamcode.managers.manipulation.ManipulationManager;
import org.firstinspires.ftc.teamcode.managers.movement.MovementManager;
import org.firstinspires.ftc.teamcode.managers.telemetry.TelemetryManager;


@Autonomous
public class LeoImu extends OpMode {
    private MovementManager driver;
    private ManipulationManager hands;
    private ImuManager gyro;
    BNO055IMU imu;
    boolean timerDoor = false;
    int step = 1;
    public ElapsedTime timer = new ElapsedTime();
    int delayStep = -1;
    double endTime;
    Orientation lastAngles = new Orientation();
    final double veryFirstAngle = gyro.getZOrientation();
    Rev2mDistanceSensor backDist2;
    Rev2mDistanceSensor backDist1;
    Rev2mDistanceSensor leftDist1;
    Rev2mDistanceSensor leftDist2;


    
    public void delayDwStop(double delay) {
        if (delayStep != step) {
            delayStep = step;
            endTime = timer.milliseconds() + delay;
        }
        if (timer.milliseconds() >= endTime) {
            hands.setMotorPower("dw", 0);
            step++;
        }
    }

    public void delayDriveStop(double delay) {
        if (delayStep != step) {
            delayStep = step;
            endTime = timer.milliseconds() + delay;
        }
        if (timer.milliseconds() >= endTime) {
            driver.stopDrive();
            step++;
        }
    }

    public boolean generalDelay(double delay) {
        boolean returnVar = false;
        if (!timerDoor) {
            endTime = timer.milliseconds();
            timerDoor = false;
        }
        if (timer.milliseconds() >= endTime) {
            timerDoor = true;
            returnVar = true;
            return returnVar;
        }
        return returnVar;
    }

        public void rotateToStart(float power) {
            final double initialRotation = gyro.getZOrientation();
            if (initialRotation > 0) {
                while (initialRotation > 0) {
                    driver.driveRaw(0.5f, 0.5f, 0.5f, 0.5f); /*todo: fix values*/
                }
            } else {
                while(initialRotation < 0) {
                    driver.driveRaw(0.5f, 0.5f, 0.5f, 0.5f); /*todo: fix values*/
                }
            }
        }

        public void rotate(double angle, float power) {
        //angle values: negative to 180 is left, positive to 180 is right
        //uses a zero point
            final double initialRotation = gyro.getZOrientation();
            if (angle > initialRotation){
                while (gyro.getZOrientation() < angle){
                    driver.driveRaw(0.5f, -0.5f, -0.5f,0.5f);
                }
            }
            if (angle < initialRotation){
                while (gyro.getZOrientation() > angle){
                    driver.driveRaw(-0.5f, 0.5f, 0.5f,-0.5f);
                }
            }
        }


    public Rev2mDistanceSensor smallerDist(Rev2mDistanceSensor sens1, Rev2mDistanceSensor sens2) {
        double num1, num2;
        num1 = sens1.getDistance(CM);
        num2 = sens2.getDistance(CM);
        if (num1 > num2) {
            return sens1;
        } else if (num2 > num1) {
            return sens2;
        } else {
            return sens1;
        }
    }


    public void init() {
        FeatureManager.setIsOpModeRunning(true);
        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor dw = hardwareMap.get(DcMotor.class, "dw");
        Servo ill = hardwareMap.get(Servo.class, "ill");
        Servo ilr = hardwareMap.get(Servo.class, "ilr");
        CRServo isl = hardwareMap.get(CRServo.class, "isl");
        CRServo isr = hardwareMap.get(CRServo.class, "isr");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        hands = new ManipulationManager(
                new CRServo[]{isl, isr},
                new String[]{"isl", "isr"},
                new Servo[]{ill, ilr},
                new String[]{"ill", "ilr"},
                new DcMotor[]{fl, fr, br, bl, dw},
                new String[]{"fl", "fr", "br", "bl", "dw"}
        );
        driver = new MovementManager(fl, fr, br, bl);
        telemetry = new TelemetryManager(telemetry, this, TelemetryManager.BITMASKS.NONE);
        gyro = new ImuManager(imu);
        driver.setDirection();
        while (!imu.isGyroCalibrated()) {
            //wait
        }






    }

    public void loop() {
        switch(step) {
            case(1):
                final float initialRotation = gyro.getZOrientation();
                telemetry.addData("rotation", gyro.getZOrientation());
            case(2):
                telemetry.addLine("Rotation");
                telemetry.addLine("rotation");
                telemetry.addData("Current rotation", gyro.getZOrientation());
                telemetry.addData("Y Acceleration", gyro.getLinearAcceleration().yAccel);
                telemetry.addData("X Acceleration", gyro.getLinearAcceleration().yAccel);
                telemetry.update();
        }


    }

}
