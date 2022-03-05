package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.managers.FeatureManager;
import org.firstinspires.ftc.teamcode.managers.manipulation.ManipulationManager;
import org.firstinspires.ftc.teamcode.managers.movement.MovementManager;
import org.firstinspires.ftc.teamcode.managers.telemetry.TelemetryManager;


@Autonomous
public class BlueFreightWarehouse extends OpMode {
    private MovementManager driver;
    private ManipulationManager hands;
    Rev2mDistanceSensor backDist2;
    Rev2mDistanceSensor backDist1;
    Rev2mDistanceSensor leftDist1;
    Rev2mDistanceSensor leftDist2;
    Rev2mDistanceSensor rightDist1;
    Rev2mDistanceSensor rightDist2;
    int step = 1;
    public ElapsedTime timer = new ElapsedTime(); ;
    int delayStep = -1;
    double endTime = timer.milliseconds();
    double initDist1;
    double initDist2;



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

    public void driveToDist(Rev2mDistanceSensor sensor, float power, double cm) {
        driver.driveRaw(power, power, power, power);
        if (sensor.getDistance(CM) <= cm) {
            driver.stopDrive();
        }

    }
    public void delayIntakeStop(double delay) {
        if (delayStep != step) {
            delayStep = step;
            endTime = timer.milliseconds() + delay;
        }
        if (timer.milliseconds() >= endTime) {
            hands.setServoPower("isl", 0);
            hands.setServoPower("isr", 0);
            step++;
        }
    }
    public void delay(double delay) {
        if (delayStep != step) {
            delayStep = step;
            endTime = timer.milliseconds() + delay;
        }
        if (timer.milliseconds() >= endTime) {
            step++;
        }
    }

    public Rev2mDistanceSensor smallerDist(Rev2mDistanceSensor sens1, Rev2mDistanceSensor sens2) {
        double num1, num2;
        num1 = sens1.getDistance(CM);
        num2 = sens2.getDistance(CM);
        if (num1>num2) {
            return sens1;
        }else if (num2>num1) {
            return sens2;
        }else {
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
        hands = new ManipulationManager(
                new CRServo[] {isl, isr},
                new String[] {"isl", "isr"},
                new Servo[] {ill, ilr},
                new String[] {"ill", "ilr"},
                new DcMotor[] {fl, fr, br, bl, dw},
                new String[] {"fl", "fr", "br", "bl", "dw"}
        );
        driver = new MovementManager(fl, fr, br, bl);
        telemetry = new TelemetryManager(telemetry, this, TelemetryManager.BITMASKS.NONE);
        driver.setDirection();
        hands.setServoPosition("ill", 0.7);
        hands.setServoPosition("ilr", 0.3);
        backDist1 = hardwareMap.get(Rev2mDistanceSensor.class, "backDist1");
        backDist2 = hardwareMap.get(Rev2mDistanceSensor.class, "backDist2");
        leftDist1 = hardwareMap.get(Rev2mDistanceSensor.class, "leftDist1");
        leftDist2 = hardwareMap.get(Rev2mDistanceSensor.class, "leftDist2");
        rightDist2 = hardwareMap.get(Rev2mDistanceSensor.class, "rightDist2");
        rightDist1 = hardwareMap.get(Rev2mDistanceSensor.class, "rightDist1");
        telemetry.addData("back cm", "%.2f cm", backDist1.getDistance(CM));
        telemetry.addData("front cm", "%.2f cm", backDist2.getDistance(CM));
        telemetry.addData("dw encoder value", hands.getPosition("dw"));



    }
    public void loop() {
        switch (step) {
            case(1):
                hands.setServoPosition("ill", 0.5);
                hands.setServoPosition("ilr", 0.5);
                delay(1000);
                step++;
                break;
            case(2):
                driver.driveRaw(0.25f,0.25f,0.25f,0.25f);
                if ((backDist1.getDistance(CM) < 300 || backDist2.getDistance(CM) < 300) && (backDist1.getDistance(CM) > 40 || backDist2.getDistance(CM) > 40)) {
                    driver.stopDrive();
                    step++;
                }
                break;
            case(3):
                hands.setServoPower("isl", 1);
                hands.setServoPower("isr", -1);
                delayIntakeStop(200);
                break;
                //TODO add delay for 200
            case(4):
                hands.setServoPower("isl", 1);
                hands.setServoPower("isr", -1);
                delayIntakeStop(200);
                break;
                //todo add delay for 200
            case(5):
                hands.setServoPower("isl", 1);
                hands.setServoPower("isr", -1);
                delayIntakeStop(200);
                break;
            case(6):
                driver.driveRaw(-0.25f,-0.25f,-0.25f,-0.25f);
                if (backDist1.getDistance(CM) <= 8 || backDist2.getDistance(CM) <= 8) {
                    driver.stopDrive();
                    step++;
                }
                break;
            case(7):
                driver.testDriveOmni(-0.25,-0.5,0);
                delayDriveStop(2000);
                break;
            case(8):
                hands.setServoPosition("ill", 0.6);
                hands.setServoPosition("ilr", 0.4);
                driver.driveRaw(0.25f,0.25f,0.25f,0.25f);
                delayDriveStop(750);
                break;
            case(9):
                telemetry.addLine("Autonomous Complete");
                telemetry.addData("time", timer.milliseconds());
                telemetry.addData("Step #", step);
                telemetry.update();
                step++;
                break;
        }
        telemetry.addLine("Encoder Values");
        telemetry.addData("fl pos", driver.flGetTicks());
        telemetry.addData("fr pos", driver.frGetTicks());
        telemetry.addData("bl pos", driver.blGetTicks());
        telemetry.addData("br pos", driver.brGetTicks());
        //Note: Commented out several lines with errors, I don't think .rawUltrasonic, .rawOptical, and .cmOptical exist.
        //telemetry.addData("front raw ultrasonic", frontDist.rawUltrasonic()); //ultrasonic data
        //telemetry.addData("front raw optical", frontDist.rawOptical()); //optical data
        //telemetry.addData("front cm optical", "%.2f cm", frontDist.cmOptical()); //cm distance? todo learn more
        //telemetry.addData("front cm", "%.2f cm", frontDist.getDistance(DistanceUnit.CM)); //cm distance
        //telemetry.addData("back raw ultrasonic", backDist.rawUltrasonic()); //ultrasonic data
        //telemetry.addData("back raw optical", backDist.rawOptical()); //optical data
        //telemetry.addData("back cm optical", "%.2f cm", backDist.cmOptical()); //cm distance? todo learn more
        telemetry.addData("back cm", "%.2f cm", backDist2.getDistance(CM)); //cm distance
        telemetry.addData("front cm", "%.2f cm", backDist1.getDistance(CM));
        telemetry.update();
    }
}
