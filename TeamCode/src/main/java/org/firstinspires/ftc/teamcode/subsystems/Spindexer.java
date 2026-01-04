package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Spindexer extends Subsystem {

    /* ===== Hardware ===== */
    private final DcMotorEx motor;
    private final ColorSensor colorSensor;
    private final Telemetry telemetry;

    /* ===== Motor Tunables ===== */
    public static double SPIN_POWER = 0.6;   // higher to overcome friction
    public static int TICKS_PER_REV = 1700;

    /* ===== HSV Tunables ===== */
    public static double GREEN_H_MIN = 133;
    public static double GREEN_H_MAX = 170;
    public static double PURPLE_H_MIN = 171;
    public static double PURPLE_H_MAX = 220;

    /* ===== State ===== */
    private Mode mode = Mode.IDLE;
    private int targetPosition = 0;

    private final float[] hsv = new float[3];

    /* ===== Ball Storage (FIFO) ===== */
    public enum BallColor { GREEN, PURPLE }
    private final BallColor[] balls = new BallColor[3];
    private int ballCount = 0;

    public enum Mode {
        IDLE,
        TO_ENCODER
    }

    public Spindexer(String motorName, String colorSensorName, HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        motor = hwMap.get(DcMotorEx.class, motorName);
        colorSensor = hwMap.get(ColorSensor.class, colorSensorName);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /* ===== Intake / Record ===== */
    public void recordBall() {
        if (ballCount >= 3) return;
        updateHSV();
        if (seesGreen()) balls[ballCount++] = BallColor.GREEN;
        else if (seesPurple()) balls[ballCount++] = BallColor.PURPLE;
    }

    /* ===== Rotate / Index ===== */
    // ================= Ball Shooting & Cataloguing =================

    // Shoot first ball (CW rotation), remove from queue if it exists
    public void shoot() {
        rotateByFraction(-1.0 / 3.0); // CW
        if (ballCount > 0) shiftLeft();
    }

    // Shoot green only if first ball is green
    public void shootGreen() {
        if (ballCount > 0 && balls[0] == BallColor.GREEN) shoot();
    }

    // Shoot purple only if first ball is purple
    public void shootPurple() {
        if (ballCount > 0 && balls[0] == BallColor.PURPLE) shoot();
    }

    // Catalogue a ball (rotate CCW), do NOT record anything — color is already stored at intake
    public void catalogue() {
        rotateByFraction(1.0 / 3.0); // CCW rotation
    }

    /* ===== Encoder Motion ===== */
    public void rotateByFraction(double fraction) {
        if (fraction == 0) return;

        int ticks = (int)(TICKS_PER_REV * fraction);
        targetPosition = motor.getCurrentPosition() + ticks;

        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(SPIN_POWER);
        mode = Mode.TO_ENCODER;

        telemetry.addData("Rotate Fraction", fraction);
        telemetry.addData("Target Pos", targetPosition);
        telemetry.addData("Current Pos", motor.getCurrentPosition());
        telemetry.addData("SPIN POWER", SPIN_POWER);
        telemetry.update();
    }

    public void stop() {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mode = Mode.IDLE;
    }

    private void shiftLeft() {
        for (int i = 0; i < ballCount - 1; i++) balls[i] = balls[i+1];
        balls[--ballCount] = null;
    }

    /* ===== Color Detection ===== */
    private void updateHSV() {
        int r = Math.min(colorSensor.red(), 255);
        int g = Math.min(colorSensor.green(), 255);
        int b = Math.min(colorSensor.blue(), 255);
        Color.RGBToHSV(r, g, b, hsv);
    }

    private boolean seesGreen() { return hsv[0] > GREEN_H_MIN && hsv[0] < GREEN_H_MAX; }
    private boolean seesPurple() { return hsv[0] > PURPLE_H_MIN && hsv[0] < PURPLE_H_MAX; }

    /* ===== Update Loop ===== */
    @Override
    public void update() {
        if (mode == Mode.TO_ENCODER && !motor.isBusy()) stop();
    }

    public boolean isIdle() { return mode == Mode.IDLE; }

    /* ===== Telemetry ===== */
    @Override
    public void addTelemetry() {
        telemetry.addData("Ball Count", ballCount);
        telemetry.addData("Ball 0", ballCount>0 ? balls[0] : "EMPTY");
        telemetry.addData("Ball 1", ballCount>1 ? balls[1] : "EMPTY");
        telemetry.addData("Ball 2", ballCount>2 ? balls[2] : "EMPTY");
        telemetry.addData("Motor Pos", motor.getCurrentPosition());
        telemetry.addData("Mode", mode);
    }
}
