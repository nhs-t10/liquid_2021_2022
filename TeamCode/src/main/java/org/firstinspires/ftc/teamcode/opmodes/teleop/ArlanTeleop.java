package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import org.firstinspires.ftc.teamcode.managers.input.InputManager;
import org.firstinspires.ftc.teamcode.managers.input.nodes.ButtonNode;
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
public class ArlanTeleop extends OpMode {
    private MovementManager driver;
    private ManipulationManager hands;
    private InputManager input;
    private ImuManager gyro;
    private boolean rintakeRunning = false;
    private boolean lintakeRunning = false;
    private boolean rBumperDown = false;
    private boolean lBumperDown = false;
    Rev2mDistanceSensor backDist2;
    Rev2mDistanceSensor backDist1;
    Rev2mDistanceSensor leftDist1;
    Rev2mDistanceSensor leftDist2;
    int step = 1;
    int delayStep = -1;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    Orientation angles = new Orientation();
    double globalAngle, correction;
    double endTime;
    float power = 0.50f;


    ElapsedTime timer;
    public void delay(double delay) {
        double endTime = timer.milliseconds() + delay;
        while (timer.milliseconds() <= endTime) {
            //do nothing
        }
    }

    public void rotateToStart(float power) {
        final double initialRotation = gyro.getZOrientation();
        if (initialRotation > 0) {
            while (initialRotation > 0) {
                driver.driveRaw(0.5f, 0.5f, 0.5f, 0.5f); /*todo: fix values*/
            }
        } else if (initialRotation < 0) {
            while(initialRotation < 0) {
                driver.driveRaw(0.5f, 0.5f, 0.5f, 0.5f); /*todo: fix values*/
            }
        }
    }



    /*private void rotate ( int degrees, float power){
        float leftPower, rightPower;
        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor dw = hardwareMap.get(DcMotor.class, "dw");
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

            while (getAngle() > degrees) {
            }
        } else    // left turn.
            while (getAngle() == 0) {
            }
        while (getAngle() < degrees) {
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
*/

    @Override
    public void init() {
        /* Phone is labelled as Not Ready For Use */
        FeatureManager.setIsOpModeRunning(true);
        timer = new ElapsedTime();
        telemetry = new TelemetryManager(telemetry, this, TelemetryManager.BITMASKS.NONE);
        gyro = new ImuManager(imu);

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

        input.registerInput("toggleTray",
                new ButtonNode("left_bumper")
        );
        input.registerInput("toggleIn",
                new ButtonNode("right_bumper")
        );
        input.registerInput("duckWheelRight",
                new ButtonNode("right_trigger")
        );
        input.registerInput("duckWheelLeft",
                new ButtonNode("left_trigger")
        );
        input.registerInput("spin",
                new ButtonNode("a")
        );
        input.registerInput("wallTouch",
                new ButtonNode(/*todo*/"")
        );
        driver.setDirection();
        gyro.getZOrientation();
    }

    @Override
    public void loop() {
        input.update();
        telemetry.addLine("l Stick Values");
        telemetry.addData("lStickX", gamepad1.left_stick_x);
        telemetry.addData("lStickY", gamepad1.left_stick_y);
        driver.testDriveOmni(gamepad1.left_stick_y/1.5, -gamepad1.left_stick_x/1.5, -gamepad1.right_stick_x/2.0);


        final double initDist1 = backDist1.getDistance(CM);
        final double initDist2 = backDist2.getDistance(CM);

        if (gamepad1.right_trigger > 0f) {
            hands.setMotorPower("dw", -1);
        } else if (gamepad1.right_trigger == 0f) {
            hands.setMotorPower("dw", 0);
        }
        if (gamepad1.left_trigger > 0f) {
            hands.setMotorPower("dw", 1);
        } else if (gamepad1.left_trigger == 0f){
            hands.setMotorPower("dw", 0);
        }

        /*
        // Toggle input motors
        if (gamepad1.right_bumper && !rBumperDown) {
            rBumperDown = true;
            rintakeRunning = !rintakeRunning;
        } else if (!gamepad1.right_bumper && rBumperDown) {
            rBumperDown = false;
        }
        if (gamepad1.left_bumper && !lBumperDown) {
            lBumperDown = true;
            lintakeRunning = !lintakeRunning;
        } else if (!gamepad1.left_bumper && lBumperDown) {
            lBumperDown = false;
        }
        if (lintakeRunning) {
            hands.setServoPower("isl", 1);
            hands.setServoPower("isr", -1);
        } else if (!lintakeRunning && !rintakeRunning) {
            hands.setServoPower("isl", 0);
            hands.setServoPower("isr", 0);
        } else if (rintakeRunning) {
            hands.setServoPower("isl", -1);
            hands.setServoPower("isr", 1);
        }

         */
        if (gamepad1.left_bumper) {
            hands.setServoPower("isl", 0.5);
            hands.setServoPower("isr", -0.5);
        } else if (gamepad1.right_bumper) {
            hands.setServoPower("isl", -1);
            hands.setServoPower("isr", 1);
        } else if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
            hands.setServoPower("isl", 0);
            hands.setServoPower("isr", 0);
        }
        if (gamepad1.b) {
            hands.setServoPosition("ill", 0.95);
            hands.setServoPosition("ilr", 0.05);
        }
        if (gamepad1.x) {
            hands.setServoPosition("ill", 0.65);
            hands.setServoPosition("ilr", 0.35);
        }
        if (gamepad1.a) {
            hands.setServoPosition("ill", 0.85);
            hands.setServoPosition("ilr", 0.15);
        }


        if (gamepad1.y/*todo*/) {
            rotateToStart(power);
            while (backDist1.getDistance(CM) <= 3 || backDist2.getDistance(CM) <= 3) {
                driver.driveRaw(-0.5f, -0.5f, -0.5f, -0.5f);
            }
        }
        if (gamepad1.dpad_down) {
            rotate(180,0.5f);
        }
        if (gamepad1.dpad_up) {
            rotate(0,0.5f);
        }
        if (gamepad1.dpad_left) {
            rotate(-90,0.5f);
        }
        if (gamepad1.dpad_right) {
            rotate(90,0.5f);
        }


        telemetry.addLine("Encoder Values");
        telemetry.addData("fl pos", driver.flGetTicks());
        telemetry.addData("fr pos", driver.frGetTicks());
        telemetry.addData("bl pos", driver.blGetTicks());
        telemetry.addData("br pos", driver.brGetTicks());
        telemetry.update();
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
