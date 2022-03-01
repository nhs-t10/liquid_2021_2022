package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.managers.FeatureManager;
import org.firstinspires.ftc.teamcode.managers.input.InputManager;
import org.firstinspires.ftc.teamcode.managers.input.nodes.JoystickNode;
import org.firstinspires.ftc.teamcode.managers.input.nodes.MultiInputNode;
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
public class TimeTesterTeleop extends OpMode {
    private MovementManager driver;
    private ManipulationManager hands;
    private InputManager input;
    int delayStep = -1;
    int step = 1;
    int bstep = 1;
    int rstep = 1;
    int lstep = 1;
    int endTime;
    boolean aButton = false; // backward
    boolean bButton = false; // right
    boolean xButton = false; // left
    boolean yButton = false; // forward
    boolean forwardTest = false; // y
    boolean backwardTest = false; // a
    boolean leftTest = false; // x
    boolean rightTest = false; // b
    int mostRecentTime = 0;
    boolean timerIncrease = false;
    boolean timerDecrease = false;

    ElapsedTime timer = new ElapsedTime();

    public void delay(int delay) {
        endTime = (int) (timer.milliseconds() + delay);
        while (timer.milliseconds() <= endTime) {
            //do nothing
        }
    }

    public void delayDriveStop(double delay) {
        if (delayStep != step) {
            delayStep = step;
            endTime = (int) (timer.milliseconds() + delay);
        }
        if (timer.milliseconds() >= endTime) {
            driver.stopDrive();
            step++;
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

        input = new InputManager(gamepad1, gamepad2);

        input.registerInput("drivingControls",
                new MultiInputNode(
                        new JoystickNode("left_stick_y"),
                        new JoystickNode("left_stick_x"),
                        new JoystickNode("right_stick_x"),
                        new JoystickNode("right_stick_y")
                )
        );
        driver.setDirection();
    }

    @Override
    public void loop() {
        input.update();
        telemetry.addLine("l Stick Values");
        telemetry.addData("lStickX", gamepad1.left_stick_x);
        telemetry.addData("lStickY", gamepad1.left_stick_y);
        driver.testDriveOmni(gamepad1.left_stick_y / 1.5, -gamepad1.left_stick_x / 1.5, -gamepad1.right_stick_x / 2.0);

        //Backward Tester
        if (gamepad1.a && !aButton) {
            aButton = true;
            backwardTest = !backwardTest;
            step = 1;
        } else if (!gamepad1.a && aButton) {
            aButton = false;
        }

        if (backwardTest) {
            switch(step) {
                case(1):
                    telemetry.addLine("Choose Time");
                    if (gamepad1.right_bumper && !timerIncrease) {
                        timerIncrease = true;
                        mostRecentTime += 50;
                    } else if (!gamepad1.right_bumper && timerIncrease) {
                        timerIncrease = false;
                    }
                    telemetry.addData("Forward Time", mostRecentTime);
                    telemetry.addLine("Press right trigger to go on");
                    if (gamepad1.right_trigger > 0f) {
                        step = 2;
                    }
                    break;
                case(2):
                    driver.driveRaw(0.5f,0.5f,0.5f,0.5f);
                    delayDriveStop(mostRecentTime);
                    break;
                case(3):
                    telemetry.addLine("Final Results");
                    telemetry.addData("Most recent time", mostRecentTime);
                    telemetry.addLine("press right trigger to exit");
                    if (gamepad1.right_trigger > 0f) {
                        telemetry.clearAll();
                        step = 4;
                    }
                    break;
            }
        }

        //Forward Tester
        if (gamepad1.y && !yButton) {
            yButton = true;
            forwardTest = !forwardTest;
            bstep = 1;
        } else if (!gamepad1.y && yButton) {
            yButton = false;
        }
        if (backwardTest) {
            switch(bstep) {
                case(1):
                    telemetry.addLine("Choose Time");
                    if (gamepad1.right_bumper && !timerIncrease) {
                        timerIncrease = true;
                        mostRecentTime += 50;
                    } else if (!gamepad1.right_bumper && timerIncrease) {
                        timerIncrease = false;
                    }
                    telemetry.addData("Backward Time", mostRecentTime);
                    telemetry.addLine("Press right trigger to go on");
                    if (gamepad1.right_trigger > 0f) {
                        bstep = 2;
                    }
                    break;
                case(2):
                    driver.driveRaw(-0.5f,-0.5f,-0.5f,-0.5f);
                    delayDriveStop(mostRecentTime);
                    break;
                case(3):
                    telemetry.addLine("Final Results");
                    telemetry.addData("Most recent time", mostRecentTime);
                    telemetry.addLine("press right trigger to exit");
                    if (gamepad1.right_trigger > 0f) {
                        telemetry.clearAll();
                        bstep = 4;
                    }
                    break;
            }
        }
        //right Tester
        if (gamepad1.b && !bButton) {
            bButton = true;
            rightTest = !rightTest;
            rstep = 1;
        } else if (!gamepad1.b && bButton) {
            bButton = false;
        }
        if (rightTest) {
            switch(rstep) {
                case(1):
                    telemetry.addLine("Choose Time");
                    if (gamepad1.right_bumper && !timerIncrease) {
                        timerIncrease = true;
                        mostRecentTime += 50;
                    } else if (!gamepad1.right_bumper && timerIncrease) {
                        timerIncrease = false;
                    }
                    telemetry.addData("Backward Time", mostRecentTime);
                    telemetry.addLine("Press right trigger to go on");
                    if (gamepad1.right_trigger > 0f) {
                        rstep = 2;
                    }
                    break;
                case(2):
                    driver.driveRaw(-0.5f,-0.5f,-0.5f,-0.5f);
                    delayDriveStop(mostRecentTime);
                    break;
                case(3):
                    telemetry.addLine("Final Results");
                    telemetry.addData("Most recent time", mostRecentTime);
                    telemetry.addLine("press right trigger to exit");
                    if (gamepad1.right_trigger > 0f) {
                        telemetry.clearAll();
                        rstep = 4;
                    }
                    break;
            }
        }
       //left Tester
        if (gamepad1.x && !xButton) {
            xButton = true;
            leftTest = !leftTest;
            lstep = 1;
        } else if (!gamepad1.x && xButton) {
            xButton = false;
        }
        if (leftTest) {
            switch(lstep) {
                case(1):
                    telemetry.addLine("Choose Time");
                    if (gamepad1.right_bumper && !timerIncrease) {
                        timerIncrease = true;
                        mostRecentTime += 50;
                    } else if (!gamepad1.right_bumper && timerIncrease) {
                        timerIncrease = false;
                    }
                    telemetry.addData("Backward Time", mostRecentTime);
                    telemetry.addLine("Press right trigger to go on");
                    if (gamepad1.right_trigger > 0f) {
                        lstep = 2;
                    }
                    break;
                case(2):
                    driver.driveRaw(-0.5f,-0.5f,-0.5f,-0.5f);
                    delayDriveStop(mostRecentTime);
                    break;
                case(3):
                    telemetry.addLine("Final Results");
                    telemetry.addData("Most recent time", mostRecentTime);
                    telemetry.addLine("press right trigger to exit");
                    if (gamepad1.right_trigger > 0f) {
                        telemetry.clearAll();
                        lstep = 4;
                    }
                    break;
            }
        }






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
