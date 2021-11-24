package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.managers.FeatureManager;
import org.firstinspires.ftc.teamcode.managers.input.InputManager;
import org.firstinspires.ftc.teamcode.managers.input.nodes.ButtonNode;
import org.firstinspires.ftc.teamcode.managers.input.nodes.InputManagerInputNode;
import org.firstinspires.ftc.teamcode.managers.input.nodes.JoystickNode;
import org.firstinspires.ftc.teamcode.managers.input.nodes.MultiInputNode;
import org.firstinspires.ftc.teamcode.managers.manipulation.ManipulationManager;
import org.firstinspires.ftc.teamcode.managers.movement.MovementManager;
import org.firstinspires.ftc.teamcode.managers.telemetry.TelemetryManager;
import org.firstinspires.ftc.teamcode.unitTests.dummy.DummyGamepad;
import org.firstinspires.ftc.teamcode.unitTests.dummy.DummyHardwareMap;
import org.firstinspires.ftc.teamcode.unitTests.dummy.DummyTelemetry;
import org.junit.Test;

@TeleOp
public class ExampleTeleop extends OpMode {
    private MovementManager driver;
    private ManipulationManager hands;
    private InputManager input;
    private boolean precision = false;
    double servoPos = 0;
    ElapsedTime timer;
    public void delay(double delay) {
        double endTime = timer.milliseconds() + delay;
        while (timer.milliseconds() <= endTime) {

        }

    }


    @Override
    public void init() {
        /* Phone is labelled as Not Ready For Use */
        FeatureManager.setIsOpModeRunning(true);
        float [] omniValues = new float [4];
        telemetry = new TelemetryManager(telemetry, this, TelemetryManager.BITMASKS.NONE);
        ElapsedTime timer;



        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor dw = hardwareMap.get(DcMotor.class, "dw");
        DcMotor is = hardwareMap.get(DcMotor.class, "is");
        Servo it = hardwareMap.get(Servo.class, "it");

        driver = new MovementManager(fl, fr, br, bl);


        hands = new ManipulationManager(new CRServo[] {}, new String[] {}, new Servo[] {it}, new String[] {"it"}, new DcMotor[] {fl, fr, br, bl, dw, is}, new String[] {"fl", "fr", "br", "bl", "dw", "is"});

        input = new InputManager(gamepad1, gamepad2);

        input.registerInput("drivingControls",
                new MultiInputNode(
                        new JoystickNode("left_stick_y"),
                        new JoystickNode("left_stick_x"),
                        new JoystickNode("right_stick_x"),
                        new JoystickNode("right_stick_y")
                )
        );


        input.registerInput("intakeSpin",
                new ButtonNode("a")
        );
        input.registerInput("duckWheelRight",
                new ButtonNode("b")
        );

        input.registerInput("duckWheelLeft",
                new ButtonNode("x")
        );

        input.registerInput("taunts",
                new MultiInputNode(
                        new ButtonNode("dpad_up"),
                        new ButtonNode("dpad_left"),
                        new ButtonNode("dpad_right"),
                        new ButtonNode("dpad_down")
                )
        );
    }

    @Override
    public void loop() {
        input.update();
        ElapsedTime timer;

        driver.downScale(0.5f);
        driver.testDriveOmni(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x/2f);

        if (input.getBool("duckWheelRight")) {
            hands.setMotorPower("dw", -0.5);
            if (gamepad1.left_bumper) {
                hands.setMotorPower("dw", -1);
            }
        }
        else if (input.getBool("duckWheelLeft")) {
            hands.setMotorPower("dw", 0.5);
            if (gamepad1.left_bumper) {
                hands.setMotorPower("dw", 1);
            }
        }
        else {
            hands.setMotorPower("dw", 0);
        }
        if (input.getBool("intakeSpin")) {
            hands.setMotorPower("is", 1);
        }
        else {
            hands.setMotorPower("is", 0);
        }
        if (gamepad1.y) {

            hands.setServoPosition("it", -0.5);
        }
        else if (gamepad1.right_bumper) {
            hands.setServoPosition("it", 0.2);

        }

        //driver.driveOmni(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);




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
