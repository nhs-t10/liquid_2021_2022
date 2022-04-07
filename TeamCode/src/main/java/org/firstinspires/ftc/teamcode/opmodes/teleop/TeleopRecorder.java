package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auxilary.RohanMath;
import org.firstinspires.ftc.teamcode.managers.FeatureManager;
import org.firstinspires.ftc.teamcode.managers.manipulation.ManipulationManager;
import org.firstinspires.ftc.teamcode.managers.movement.MovementManager;
import org.firstinspires.ftc.teamcode.managers.telemetry.TelemetryManager;
import org.firstinspires.ftc.teamcode.unitTests.dummy.DummyGamepad;
import org.firstinspires.ftc.teamcode.unitTests.dummy.DummyHardwareMap;
import org.firstinspires.ftc.teamcode.unitTests.dummy.DummyTelemetry;
import org.junit.Test;

/*
 * A: Turn 180
 * LB: Toggle Servo
 * RB: Toggle Intake Motors
 * L2: Duck Wheel Left
 * R2: Duck Wheel Right
 * LStick: Omni
 * RStick: X: turn
 */

@TeleOp
public class TeleopRecorder extends OpMode {
    private MovementManager driver;
    private ManipulationManager hands;
    public ElapsedTime timer = new ElapsedTime();
    double endTime;
    int fStep = 1;
    int bStep = 1;
    int lStep = 1;
    int rStep = 1;

    int delayStep = -1;
    int miniStep = 1;
    double[] driverDirection;

    public void delayDriveStop(double delay) {
        if (delayStep != miniStep) {
            delayStep = miniStep;
            endTime = timer.milliseconds() + delay;
        }
        if (timer.milliseconds() >= endTime) {
            driver.stopDrive();
            miniStep++;
        }
    }
    public void delayIntakeStop(double delay) {
        if (delayStep != miniStep) {
            delayStep = miniStep;
            endTime = timer.milliseconds() + delay;
        }
        if (timer.milliseconds() >= endTime) {
            hands.setServoPower("isl", 0);
            hands.setServoPower("isr", 0);
            miniStep++;
        }
    }
    @Override
    public void init() {
        /* Phone is labelled as Not Ready For Use */
        FeatureManager.setIsOpModeRunning(true);
        telemetry = new TelemetryManager(telemetry, this, TelemetryManager.BITMASKS.NONE);

        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor dw = hardwareMap.get(DcMotor.class, "dw");
        CRServo isl = hardwareMap.get(CRServo.class, "isl");
        CRServo isr = hardwareMap.get(CRServo.class, "isr");
        Servo ill = hardwareMap.get(Servo.class, "ill");
        Servo ilr = hardwareMap.get(Servo.class, "ilr");


        driver = new MovementManager(fl, fr, br, bl);

        hands = new ManipulationManager(
                new CRServo[] {isl, isr},
                new String[] {"isl", "isr"},
                new Servo[] {ill, ilr},
                new String[] {"ill", "ilr"},
                new DcMotor[] {fl, fr, br, bl, dw},
                new String[] {"fl", "fr", "br", "bl", "dw"}
        );



        driver.setDirection();
    }

    @Override

    public void loop() {
        if (gamepad1.dpad_up) {
            driver.driveRaw(0.5f,0.5f,0.5f,0.5f);
            switch(fStep) {
                case(1):
                    timer.reset();
                    fStep++;
                    break;
                case(2):
                    telemetry.addData("Forward time: ", timer.milliseconds());
            }
        } else if (gamepad1.dpad_left) {
            driver.testDriveOmni(0,-0.5,0);
            switch(lStep) {
                case(1):
                    timer.reset();
                    lStep++;
                    break;
                case(2):
                    telemetry.addData("Left time: ", timer.milliseconds());
            }
        } else if (gamepad1.dpad_down) {
            driver.driveRaw(-0.5f,-0.5f,-0.5f,-0.5f);
            switch(bStep) {
                case(1):
                    timer.reset();
                    bStep++;
                    break;
                case(2):
                    telemetry.addData("Backward time: ", timer.milliseconds());
            }
        } else if (gamepad1.dpad_right) {
            driver.testDriveOmni(0, 0.5, 0);
            switch(rStep) {
                case(1):
                    timer.reset();
                    rStep++;
                    break;
                case(2):
                    telemetry.addData("Right Time: ", timer.milliseconds());
            }
        } else {
            driver.stopDrive();
            fStep = 1;
            bStep = 1;
            rStep = 1;
            lStep = 1;
        }



//        if (driverDirection[0] != 0) {
//            switch(step) {
//                case(1):
//                    final double initTime = timer.milliseconds();
//                    step++;
//                    break;
//                case(2):
//
//            }
//        }

    }
    public void stop() {
        FeatureManager.setIsOpModeRunning(false);
    }

    @Test
    public void test() {
        this.hardwareMap = new DummyHardwareMap();
        this.gamepad1 = new DummyGamepad();
        this.gamepad2 = new DummyGamepad();
        this.telemetry = new DummyTelemetry();

        long startTime = System.currentTimeMillis();

        this.start();
        this.init();
        this.init_loop();
        for(int i = 0; i < 3; i++) {
            this.time = (System.currentTimeMillis() - startTime) * 0.001;
            this.loop();
        }
        this.stop();
    }
}
