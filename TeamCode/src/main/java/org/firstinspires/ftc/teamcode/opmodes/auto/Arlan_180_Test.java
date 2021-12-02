package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.managers.FeatureManager;
import org.firstinspires.ftc.teamcode.managers.manipulation.ManipulationManager;
import org.firstinspires.ftc.teamcode.managers.movement.MovementManager;
import org.firstinspires.ftc.teamcode.managers.telemetry.TelemetryManager;


@Autonomous
public class Arlan_180_Test extends OpMode {
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
        hands = new ManipulationManager(
                new CRServo[] {},
                new String[] {},
                new Servo[] {},
                new String[] {},
                new DcMotor[] {fl, fr, br, bl, dw},
                new String[] {"fl", "fr", "br", "bl", "dw"}
        );
        driver = new MovementManager(fl, fr, br, bl);
        telemetry = new TelemetryManager(telemetry, this, TelemetryManager.BITMASKS.NONE);
        driver.setDirection();



    }
    public void loop() {
        switch (step) {
            case(1):
                timer = new ElapsedTime();
                //Make sure to add this line in each "case"
                driver.resetEncoders();
                //Moves the robot for 1 unit forward
                driver.setTargetPositions(560, 560, 560, 560);
                float turnDist = 5.0f;
                driver.driveRaw(turnDist, -turnDist, turnDist, -turnDist);
                driver.setTargetPositions(-324, 324, 324, -324);
                step++;
                break;
            case(2):
                telemetry.addLine("Autonomous Complete");
                telemetry.addData("time", timer.milliseconds());
                telemetry.addData("Step #", step);
                telemetry.update();
        }
    }

}

