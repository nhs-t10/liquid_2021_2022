package org.firstinspires.ftc.teamcode.opmodes.auto;

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

/*
  ========== field data ==========
  wheel circumference = 27.98
  short move distance 39.37 from duck wheel to carousel
  long move distance 170.18 from duck wheel to carousel
  ========== goal ==========
  move from far on same side to duck wheel and then to warehouse
*/

@Autonomous
public class Auto_Long extends OpMode {
    private MovementManager driver;
    private ManipulationManager hands;
    public double circ = 27.98;
    public double smove = 39.37;
    public double lmove = 170.18;
    float [] omniValues = new float [4];
    int step = 1;
    ElapsedTime timer;

    public void delayDrive(double delay) {
        double endTime = timer.milliseconds() + delay;
        while (timer.milliseconds() <= endTime) {
            driver.driveRaw(0.75f, 0.75f, 0.75f, 0.75f);
        }
        driver.stopDrive();

    }

    public void delay(double delay) {
        double endTime = timer.milliseconds() + delay;
        while (timer.milliseconds() <= endTime) {}
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
                driver.resetEncoders();
                int move_dist = (int) (lmove / circ * 560);
                driver.setTargetPositions(move_dist, move_dist, move_dist, move_dist);
                step++;
                break;
            case(2):
                hands.setMotorPower("dw", -0.5);
                this.delay(5*1000);
                hands.setMotorPower("dw", 0);
                step++;
                break;
            case(3):
                driver.resetEncoders();
                int dist = -50 * 560;
                driver.setTargetPositions(dist, dist, dist, dist);
                step++;
                break;
            default:
                telemetry.addLine("Autonomous Complete");
                telemetry.addData("time", timer.milliseconds());
                telemetry.addData("Step #", step);
                telemetry.update();
        }
    }
}
