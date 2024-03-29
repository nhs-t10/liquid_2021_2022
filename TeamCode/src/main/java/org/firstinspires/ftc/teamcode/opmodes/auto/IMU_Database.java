package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.managers.FeatureManager;
import org.firstinspires.ftc.teamcode.managers.manipulation.ManipulationManager;
import org.firstinspires.ftc.teamcode.managers.movement.MovementManager;
import org.firstinspires.ftc.teamcode.managers.telemetry.TelemetryManager;


@Autonomous
public class IMU_Database extends OpMode {
    private MovementManager driver;
    private ManipulationManager hands;
    ModernRoboticsI2cRangeSensor rangeSensor;
    int step = 1;
    ElapsedTime timer;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;
    float power = 0.50f;
    boolean aButton, bButton;

    public void delay(double delay) {
        double endTime = timer.milliseconds() + delay;
        while (timer.milliseconds() <= endTime) {
        }
    }

    public void driveToDistanceForward(float power, double cm) {
        driver.driveRaw(power, power, power, power);
        while (rangeSensor.getDistance(DistanceUnit.CM) >= cm) {
            //wait
        }
        driver.stopDrive();
    }

    public void driveToDistanceBackward(float power, double cm) {
        driver.driveRaw(power, power, power, power);
        while (rangeSensor.getDistance(DistanceUnit.CM) <= cm) {
            //wait
        }
        driver.stopDrive();
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle () {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private double checkDirection () {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    public void rotate ( int degrees, float power){
        float leftPower, rightPower;
        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        // set power to rotate.
        driver.driveRaw(leftPower, rightPower, rightPower, leftPower);


        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (getAngle() == 0) {
            }

            while (getAngle() < degrees) {
            }
        } else    // left turn.
            while (getAngle()==0){}
            while (getAngle() > degrees) {
            }

        // turn the motors off.
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

        // wait for rotation to stop.
        delay(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }


    public void init() {
        FeatureManager.setIsOpModeRunning(true);
        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor dw = hardwareMap.get(DcMotor.class, "dw");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;


        hands = new ManipulationManager(
                new CRServo[]{},
                new String[]{},
                new Servo[]{},
                new String[]{},
                new DcMotor[]{fl, fr, br, bl, dw},
                new String[]{"fl", "fr", "br", "bl", "dw"}
        );
        driver = new MovementManager(fl, fr, br, bl);
        driver.runUsingEncoders();
        telemetry = new TelemetryManager(telemetry, this, TelemetryManager.BITMASKS.NONE);
        driver.setDirection();
        timer = new ElapsedTime();
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
    }

    public void loop() {
        switch (step) {
            case (1):
                // Use gyro to drive in a straight line.
                correction = checkDirection();


                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.update();

                driver.driveRaw(power - (float) correction,power - (float) correction,power - (float) correction,power - (float) correction);


                // We record the sensor values because we will test them in more than
                // one place with time passing between those places. See the lesson on
                // Timing Considerations to know why.

                aButton = gamepad1.a;
                bButton = gamepad1.b;

                if (aButton || bButton) {
                    // backup.
                    driver.driveRaw(power, power, power, power);

                    delay(500);

                    // stop.
                    driver.driveRaw(0, 0, 0, 0);

                    // turn 90 degrees right.
                    if (aButton) rotate(-90, power);

                    // turn 90 degrees left.
                    if (bButton) rotate(90, power);
                }
        }
    }


    }


