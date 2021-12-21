package org.firstinspires.ftc.teamcode.managers.movement;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.auxilary.PaulMath;
import org.firstinspires.ftc.teamcode.managers.FeatureManager;
import org.firstinspires.ftc.teamcode.managers.imu.ImuManager;

/*
  ========== field data ==========
  Multiply by 537.6 when using encoders!!!
  wheel circumference = 27.98
  short move distance 39.37 from duck wheel to carousel
  long move distance 170.18 from duck wheel to carousel
*/

public class MovementManager extends FeatureManager {

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    ElapsedTime timer = new ElapsedTime();
    public BNO055IMU imu;
    public ImuManager gyro = new ImuManager(imu);
    Orientation angles = gyro.getOrientation();



    private static float scale = 1f;

    /**
     * Create a MovementManager with four motors.
     * @param fl Front Left motor
     * @param fr Front Right motor
     * @param br Back Right motor
     * @param bl Back Left motor
     */
    public MovementManager(DcMotor fl, DcMotor fr, DcMotor br, DcMotor bl) {
        this.frontLeft = fl;
        this.frontRight = fr;
        this.backRight = br;
        this.backLeft = bl;

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public MovementManager(DcMotor fl, DcMotor fr, DcMotor br, DcMotor bl, ElapsedTime timer) {
        this.frontLeft = fl;
        this.frontRight = fr;
        this.backRight = br;
        this.backLeft = bl;
        this.timer = timer;

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void driveRaw(float fl, float fr, float br, float bl) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backRight.setPower(br);
        backLeft.setPower(bl);
    }

    public void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }
    public void holdUp(double delay) {
        double endTime = timer.milliseconds() + delay;
        while (timer.milliseconds() <= endTime) {
            stopDrive();
        }
    }

    public void rotate(int degrees, float power) {
        float leftSidePower = -1 * Math.signum(degrees) * power;
        float rightSidePower = Math.signum(degrees) * power;



        driveRaw(leftSidePower, rightSidePower, leftSidePower, rightSidePower);

    }

    public void timeDriveRaw(double delay, float fl, float fr, float br, float bl) {
        double endTime = timer.milliseconds() + delay;
        while (timer.milliseconds() <= endTime) {
            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backRight.setPower(br);
            backLeft.setPower(bl);
        }
        stopDrive();

    }

    public void timeDriveOmni(double delay, float vert, float rotate, float hori) {
        double endTime = timer.milliseconds() + delay;
        while (timer.milliseconds() <= endTime) {
            testDriveOmni(vert, rotate, hori);
        }
        stopDrive();
    }

    public void setDirection() {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void testDriveOmni(float y, float x, float rx) {
        double maxValue = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flPower = (y + x + rx)/maxValue;
        double blPower = (y - x + rx) / maxValue;
        double frPower = (y - x - rx) / maxValue;
        double brPower = (y + x - rx) / maxValue;

        frontLeft.setPower(flPower * scale);
        frontRight.setPower(frPower * scale);
        backRight.setPower(brPower * scale);
        backLeft.setPower(blPower * scale);
    }



    public void encoderDriveRaw (
            int flDistance,
            int frDistance,
            int blDistance,
            int brDistance, 
            float drivePower) {
        final int flStart = frontLeft.getCurrentPosition();
        final int frStart = frontRight.getCurrentPosition();
        final int blStart = backLeft.getCurrentPosition();
        final int brStart = backRight.getCurrentPosition();

        drivePower = Math.abs(drivePower);

        driveRaw(Math.signum(flDistance) * drivePower, Math.signum(frDistance) * drivePower, Math.signum(brDistance) * drivePower, Math.signum(blDistance) * drivePower);

        if (Math.abs(frontLeft.getCurrentPosition() - flStart - flDistance) < 10) {
            frontLeft.setPower(0);
        }
        if (Math.abs(frontRight.getCurrentPosition() - frStart - frDistance) < 10) {
            frontRight.setPower(0);
        }
        if (Math.abs(backLeft.getCurrentPosition() - blStart - blDistance) < 10) {
            backLeft.setPower(0);
        }
        if (Math.abs(backRight.getCurrentPosition() - brStart - brDistance) < 10) {
            backRight.setPower(0);
        }

    }

    public void encoderDriveOmni (
            float y,
            float x,
            float rx,
            int totalDistance) {
        double maxValue = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flPower = (y + x + rx) / maxValue;
        double blPower = (y - x + rx) / maxValue;
        double frPower = (y - x - rx) / maxValue;
        double brPower = (y + x - rx) / maxValue;

        resetEncoders();

        double flDiff = Math.abs(frontLeft.getCurrentPosition() - (flPower * totalDistance));
        double frDiff = Math.abs(frontRight.getCurrentPosition() - (frPower * totalDistance));
        double blDiff = Math.abs(backLeft.getCurrentPosition() - (blPower * totalDistance));
        double brDiff = Math.abs(backRight.getCurrentPosition() - (brPower * totalDistance));

        if (flDiff > 5) {
            frontLeft.setPower(Math.signum(totalDistance) * flPower);
        } else {frontLeft.setPower(0);}
        if (frDiff > 5) {
            frontRight.setPower(Math.signum(totalDistance) * frPower);
        } else {frontRight.setPower(0);}
        if (blDiff > 5) {
            backLeft.setPower(Math.signum(totalDistance) * blDiff);
        } else {backLeft.setPower(0);}
        if (brDiff > 5) {
            backRight.setPower(Math.signum(totalDistance) * brDiff);
        } else {backRight.setPower(0);}
    }

    public void runToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runUsingEncoders() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runWithOutEncoders() {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetPositions(int fl, int fr, int br, int bl) {
        frontLeft.setTargetPosition(fl);
        frontRight.setTargetPosition(fr);
        backRight.setTargetPosition(br);
        backLeft.setTargetPosition(bl);
    }



    public void driveVertical(float power, float distance) {
        int ticks = PaulMath.encoderDistance(distance);
        setTargetPositions(ticks, ticks, ticks, ticks);
        runToPosition();
        while(
                Math.abs(frontLeft.getCurrentPosition()) < Math.abs(frontLeft.getTargetPosition()) &&
                        Math.abs(frontRight.getCurrentPosition()) < Math.abs(frontRight.getTargetPosition()) &&
                        Math.abs(backRight.getCurrentPosition()) < Math.abs(backRight.getTargetPosition()) &&
                        Math.abs(backLeft.getCurrentPosition()) < Math.abs(backLeft.getTargetPosition())
        ) {
            driveRaw(power, power, power, power);
            //Waiting for motor to finish
        }
        stopDrive();
    }
    public void driveOmni(float[] powers) {
        float[] sum = PaulMath.omniCalc(powers[0]*scale, powers[1]*scale, powers[2] * scale);
        driveRaw(sum[0], sum[1], sum[2], sum[3]);
    }
    public void driveOmni(float v, float h, float r) {
        driveOmni(new float[] {v, h, r});
    }

    public void driveOmniExponential(float[] powers) {
        float[] sum = PaulMath.omniCalc(
                (float) Math.pow(powers[0], EXPONENTIAL_SCALAR),
                (float) Math.pow(powers[1], EXPONENTIAL_SCALAR),
                (float) Math.pow(powers[2], EXPONENTIAL_SCALAR));
        driveRaw(sum[0], sum[1], sum[2], sum[3]);
    }
    public DcMotor[] getMotor(){
        DcMotor[] motors = {frontLeft, frontRight, backRight, backLeft};

        return motors;
    }
    public void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public float getScale(){
        return scale;
    }
    public void upScale(float ScaleFactor){
        scale+=ScaleFactor;
    }
    public void downScale(float ScaleFactor){
        scale-=ScaleFactor;
    }

    public float[] getMotorPositions() {
        return new float[] {
          frontRight.getCurrentPosition(),
          frontLeft.getCurrentPosition(),
          backRight.getCurrentPosition(),
          backLeft.getCurrentPosition()
        };
    }

    public void driveWithVertical(float power, float distance) {
        int ticks = PaulMath.encoderDistance(distance);
        setTargetPositions(ticks, ticks, ticks, ticks);
        runUsingEncoders();
        if(Math.abs(frontLeft.getCurrentPosition()) < Math.abs(frontLeft.getTargetPosition()) &&
                        Math.abs(frontRight.getCurrentPosition()) < Math.abs(frontRight.getTargetPosition()) &&
                        Math.abs(backRight.getCurrentPosition()) < Math.abs(backRight.getTargetPosition()) &&
                        Math.abs(backLeft.getCurrentPosition()) < Math.abs(backLeft.getTargetPosition())
        ) {
            driveRaw(power, power, power, power);
            //Waiting for motor to finish
        } else {
            stopDrive();
        }

    }

    public int flGetTicks() {
        return frontLeft.getCurrentPosition();
    }
    public int frGetTicks() {
        return frontRight.getCurrentPosition();
    }
    public int blGetTicks() {
        return backLeft.getCurrentPosition();
    }
    public int brGetTicks() {
        return backRight.getCurrentPosition();
    }

    public int getHorizontalTicks() { return frontRight.getCurrentPosition(); }
    public int getVerticalTicks() { return  backLeft.getCurrentPosition(); }

}
