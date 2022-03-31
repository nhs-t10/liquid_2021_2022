package org.firstinspires.ftc.teamcode.opmodes.auto.archive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.managers.FeatureManager;
import org.firstinspires.ftc.teamcode.managers.manipulation.ManipulationManager;
import org.firstinspires.ftc.teamcode.managers.movement.MovementManager;
import org.firstinspires.ftc.teamcode.managers.telemetry.TelemetryManager;


public class Auto_Diagonal_Short_Right extends OpMode {
    private MovementManager driver;
    private ManipulationManager hands;
    float [] omniValues = new float [4];
    int step = 1;
    ElapsedTime timer;
    public void delay(double delay) {
        double endTime = timer.milliseconds() + delay;
        while (timer.milliseconds() <= endTime) {
            //do nothing
        }


    }

    public void init() {
        FeatureManager.setIsOpModeRunning(true);
        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor dw = hardwareMap.get(DcMotor.class, "dw");
        hands = new ManipulationManager(new CRServo[] {}, new String[] {}, new Servo[] {}, new String[] {}, new DcMotor[] {fl, fr, br, bl, dw}, new String[] {"fl", "fr", "br", "bl", "dw"});
        driver = new MovementManager(fl, fr, br, bl);
        telemetry = new TelemetryManager(telemetry, this, TelemetryManager.BITMASKS.NONE);

    }
    public void loop() {
        switch (step) {
            case(1):
                timer = new ElapsedTime();
                driver.resetEncoders();
                step++;
                break;
            case(2):
                timer = new ElapsedTime();
                driver.setTargetPositions(7346, 7346, 7346, 7346); /* numbers are semi-random, test these */
                driver.driveRaw(1f, 1f,1f, 1f);
                driver.resetEncoders();
                step++;
                break;
            case(3):
                timer = new ElapsedTime();
                hands.setMotorPower("dw", 0.5);
                delay(5000); /* should spin the wheel for five seconds */
                hands.setMotorPower("dw", 0);
                driver.resetEncoders();
                step++;
                break;
            case(4):
                timer = new ElapsedTime();
                driver.setTargetPositions(-2500, 2500, 2500, -2500); /* turns it some amount. untested if it actually turns the right amount */
                driver.driveRaw(0.5f, 0.5f,0.5f, 0.5f);
                driver.resetEncoders();
                driver.setTargetPositions(7321, 7321, 7321, 7321); /* distance untested */
                driver.driveRaw(1f, 1f,1f, 1f);
                driver.resetEncoders();
                step++;
                break;
            case(5):
                telemetry.addLine("Autonomous Complete");
                telemetry.addData("time", timer.milliseconds());
                telemetry.addData("Step #", step);
                telemetry.update();
        }
    }

}


