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
public class ArlanTeleop extends OpMode {
    private MovementManager driver;
    private ManipulationManager hands;
    private InputManager input;
    private boolean autoTeleop = false;
    private boolean yButton = false;
    public ElapsedTime timer = new ElapsedTime();
    double endTime;
    int delayStep = -1;
    int miniStep = 1;
    boolean timerDoor = false;

    public void generalDelay(double delay) {
        for (int i = 0; i<1; i++) {
            endTime = timer.milliseconds();
        }
        if (timer.milliseconds() >= endTime) {
            miniStep++;
        }
    }
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
            hands.setServoPower("intakeL", 0);
            hands.setServoPower("intakeR", 0);
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
        CRServo launcherL = hardwareMap.get(CRServo.class, "launcherL");
        CRServo launcherR = hardwareMap.get(CRServo.class, "launcherR");
        CRServo intakeL = hardwareMap.get(CRServo.class, "intakeL");
        CRServo intakeR = hardwareMap.get(CRServo.class, "intakeR");
        Servo lifterL = hardwareMap.get(Servo.class, "lifterL");
        Servo lifterR = hardwareMap.get(Servo.class, "lifterR");


        driver = new MovementManager(fl, fr, br, bl);

        hands = new ManipulationManager(
                new CRServo[] {intakeL, intakeR, launcherL, launcherR},
                new String[] {"intakeL", "intakeR", "launcherL", "launcherR"},
                new Servo[] {lifterL, lifterR},
                new String[] {"lifterL", "lifterR"},
                new DcMotor[] {fl, fr, br, bl,},
                new String[] {"fl", "fr", "br", "bl"}
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

/*        if (gamepad1.right_trigger > 0f) {
            hands.setMotorPower("dw", -1);
        } else if (gamepad1.right_trigger == 0f) {
            hands.setMotorPower("dw", 0);
        }
        if (gamepad1.left_trigger > 0f) {
            hands.setMotorPower("dw", 1);
        } else if (gamepad1.left_trigger == 0f){
            hands.setMotorPower("dw", 0);
        }*/

        if (gamepad1.left_bumper) {
            hands.setServoPower("launcherR", -1);
            hands.setServoPower("LauncherL", 1);
            generalDelay(5000);
        } else if (gamepad1.right_bumper) {
            hands.setServoPower("intakeL", 1);
            hands.setServoPower("intakeR", -1);
        }
        if (gamepad1.b) {
            hands.setServoPosition("lifterL", 0.25);
            hands.setServoPosition("lifterR", 0.75);
        }
        if (gamepad1.x) {
            hands.setServoPosition("lifterL", 0.55);
            hands.setServoPosition("lifterR", 0.45);
        }
        if (gamepad1.a) {
            hands.setServoPosition("lifterL", 0.35);
            hands.setServoPosition("lifterR", 0.65);
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
