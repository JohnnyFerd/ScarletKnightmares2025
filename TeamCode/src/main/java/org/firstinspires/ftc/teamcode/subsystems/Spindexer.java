package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Spindexer extends Subsystem {

    private final DcMotorEx motor;
    private final ColorSensor colorSensor;
    private final Telemetry telemetry;

    /* ===== Tunables ===== */
    public static double SPIN_POWER = 0.4;

    // encoder ticks per full revolution, measure this
    public static int TICKS_PER_REV = 1440;

    // color thresholds
    public static int GREEN_MIN = 150;
    public static int PURPLE_RED_MIN = 120;
    public static int PURPLE_BLUE_MIN = 120;

    /* ===== State ===== */
    private boolean rotateToGreen = false;
    private boolean rotateToPurple = false;
    private boolean rotateByEncoder = false;

    private int targetPosition = 0;

    public Spindexer(
            String motorName,
            String colorSensorName,
            HardwareMap hwMap,
            Telemetry telemetry
    ) {
        this.telemetry = telemetry;

        motor = hwMap.get(DcMotorEx.class, motorName);
        colorSensor = hwMap.get(ColorSensor.class, colorSensorName);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /* ===== Public Commands ===== */

    public void rotateUntilGreen() {
        rotateToGreen = true;
        rotateToPurple = false;
        rotateByEncoder = false;
    }

    public void rotateUntilPurple() {
        rotateToPurple = true;
        rotateToGreen = false;
        rotateByEncoder = false;
    }

    // example fraction = 1.0 / 3.0
    public void rotateByFraction(double fraction) {
        int ticks = (int) (TICKS_PER_REV * fraction);

        targetPosition = motor.getCurrentPosition() + ticks;

        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(SPIN_POWER);

        rotateByEncoder = true;
        rotateToGreen = false;
        rotateToPurple = false;
    }

    public void stop() {
        motor.setPower(0);
        rotateToGreen = false;
        rotateToPurple = false;
        rotateByEncoder = false;
    }

    /* ===== Color Detection ===== */

    private boolean seesGreen() {
        return colorSensor.green() > GREEN_MIN;
    }

    private boolean seesPurple() {
        return colorSensor.red() > PURPLE_RED_MIN
                && colorSensor.blue() > PURPLE_BLUE_MIN;
    }

    /* ===== Update Loop ===== */

    @Override
    public void update() {

        if (rotateToGreen) {
            if (seesGreen()) {
                stop();
            } else {
                motor.setPower(SPIN_POWER);
            }
        }

        else if (rotateToPurple) {
            if (seesPurple()) {
                stop();
            } else {
                motor.setPower(SPIN_POWER);
            }
        }

        else if (rotateByEncoder) {
            if (!motor.isBusy()) {
                motor.setPower(0);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rotateByEncoder = false;
            }
        }
    }

    /* ===== Telemetry ===== */

    @Override
    public void addTelemetry() {
        telemetry.addLine("Spindexer");
        telemetry.addData("Encoder", motor.getCurrentPosition());
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue", colorSensor.blue());
    }
}
