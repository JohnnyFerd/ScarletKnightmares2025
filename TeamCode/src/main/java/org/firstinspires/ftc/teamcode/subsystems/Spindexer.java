package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Spindexer extends Subsystem {

    private final DcMotorEx motor;
    private final ColorSensor colorSensor;
    private final Telemetry telemetry;
    private final DigitalChannel hallSensor;



    /* ===== Hall Effect State ===== */
    private boolean lastHallState = false;


    /* ===== Motor Tunables ===== */
    public static double SPIN_POWER = 0.4;
    public static int TICKS_PER_REV = 1700;

    /* ===== HSV Tunables (based on your measured data) ===== */

    // Green ≈ Hue 159, Sat ≈ 0.69
    public static double GREEN_H_MIN = 133;
    public static double GREEN_H_MAX = 170;
    public static double GREEN_S_MIN = 0.5;

    // Purple ≈ Hue 230, Sat ≈ 0.43
    public static double PURPLE_H_MIN = 170.9;
    public static double PURPLE_H_MAX = 220;
    public static double PURPLE_S_MIN = 0.35;

    // White / Black (optional, now valid since RGB is normalized)
    public static double WHITE_S_MAX = 0.25;
    public static double WHITE_V_MIN = 0.8;
    public static double BLACK_V_MAX = 0.2;

    /* ===== State ===== */
    private Mode mode = Mode.IDLE;
    private int targetPosition = 0;

    private enum Mode {
        IDLE,
        TO_GREEN,
        TO_PURPLE,
        TO_ENCODER
    }

    /* ===== HSV Cache ===== */
    private final float[] hsv = new float[3];

    public Spindexer(
            String motorName,
            String colorSensorName,
            String hallSensorName,
            HardwareMap hwMap,
            Telemetry telemetry
    ) {
        this.telemetry = telemetry;


        motor = hwMap.get(DcMotorEx.class, motorName);
        colorSensor = hwMap.get(ColorSensor.class, colorSensorName);
        hallSensor = hwMap.get(DigitalChannel.class, hallSensorName);
        hallSensor.setMode(DigitalChannel.Mode.INPUT);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /* ===== Commands ===== */

    public void rotateUntilGreen() {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mode = Mode.TO_GREEN;
    }
    public boolean isHallTriggered() {
        return !hallSensor.getState();
    }

    public void rotateUntilPurple() {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mode = Mode.TO_PURPLE;
    }

    public void rotateByFraction(double fraction) {
        int ticks = (int) (TICKS_PER_REV * fraction);
        targetPosition = motor.getCurrentPosition() + ticks;

        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(SPIN_POWER);

        mode = Mode.TO_ENCODER;
    }

    public void stop() {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mode = Mode.IDLE;
    }

    /* ===== HSV Utilities ===== */

    private void updateHSV() {
        // Normalize sensor values to 0–255 (FTC sensors exceed this)
        int r = Math.min(colorSensor.red(), 255);
        int g = Math.min(colorSensor.green(), 255);
        int b = Math.min(colorSensor.blue(), 255);

        Color.RGBToHSV(r, g, b, hsv);
    }

    public float[] getHSV() {
        updateHSV();
        return hsv;
    }

    /* ===== Color Checks ===== */

    private boolean seesGreen() {
        return hsv[0] > GREEN_H_MIN
                && hsv[0] < GREEN_H_MAX;
    }

    private boolean seesPurple() {
        return hsv[0] > PURPLE_H_MIN
                && hsv[0] < PURPLE_H_MAX;
    }

    private boolean seesWhite() {
        return hsv[1] < WHITE_S_MAX
                && hsv[2] > WHITE_V_MIN;
    }

    private boolean seesBlack() {
        return hsv[2] < BLACK_V_MAX;
    }

    /* ===== Update Loop ===== */

    @Override
    public void update() {
        updateHSV();

        switch (mode) {

            case TO_GREEN:
                if (seesGreen()) stop();
                else motor.setPower(SPIN_POWER);
                break;

            case TO_PURPLE:
                if (seesPurple()) stop();
                else motor.setPower(SPIN_POWER);
                break;

            case TO_ENCODER:
                if (!motor.isBusy()) stop();
                break;

            case IDLE:
            default:
                motor.setPower(0);
                break;
        }


        lastHallState = isHallTriggered();


    }

    /* ===== Telemetry ===== */

    @Override
    public void addTelemetry() {
        telemetry.addLine("Spindexer");
        telemetry.addData("Mode", mode);
        telemetry.addData("Encoder", motor.getCurrentPosition());
        telemetry.addData("HALL SENSOR", isHallTriggered());
        telemetry.addData("R", colorSensor.red());
        telemetry.addData("G", colorSensor.green());
        telemetry.addData("B", colorSensor.blue());

        telemetry.addData("Hue", "%.1f", hsv[0]);
        telemetry.addData("Sat", "%.2f", hsv[1]);
        telemetry.addData("Val", "%.2f", hsv[2]);
    }

    /* ===== Raw Accessors ===== */

    public int getRed() { return colorSensor.red(); }
    public int getGreen() { return colorSensor.green(); }
    public int getBlue() { return colorSensor.blue(); }
}
