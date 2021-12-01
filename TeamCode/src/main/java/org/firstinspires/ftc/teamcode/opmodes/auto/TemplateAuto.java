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
public class TemplateAuto extends OpMode {
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
                driver.setTargetPositions(0, 0, 560, 5);
                driver.runToPosition();

                driver.driveRaw(0.25f, 0.25f, 0.25f, 0.25f);
                step++;
                break;
        }
        telemetry.addData("fl enc", driver.frontLeft.getCurrentPosition());
        telemetry.addData("fr enc", driver.frontRight.getCurrentPosition());
        telemetry.addData("bl enc", driver.backLeft.getCurrentPosition());
        telemetry.addData("br enc", driver.backRight.getCurrentPosition());

        telemetry.addData("fl m", driver.frontLeft.getMode());
        telemetry.addData("fr m", driver.frontRight.getMode());
        telemetry.addData("bl m", driver.backLeft.getMode());
        telemetry.addData("br m", driver.backRight.getMode());
    }

}

