package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake extends Subsystem {

    private final DcMotorEx motor;
    private final Telemetry telemetry;

    /* ===== Tunables ===== */
    public static double INTAKE_POWER = 1;

    /* ===== State ===== */
    private Mode mode = Mode.OFF;

    private enum Mode {
        ON,
        REVERSE,
        OFF
    }

    public Intake(String motorName, HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        motor = hwMap.get(DcMotorEx.class, motorName);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /* ===== Commands ===== */

    public void intakeOn() {
        mode = Mode.ON;
    }

    public void intakeReverse() {
        mode = Mode.REVERSE;
    }

    public void intakeOff() {
        mode = Mode.OFF;
    }

    /* ===== Update Loop ===== */

    @Override
    public void update() {
        switch (mode) {
            case ON:
                motor.setPower(INTAKE_POWER);
                break;

            case REVERSE:
                motor.setPower(-INTAKE_POWER);
                break;

            case OFF:
            default:
                motor.setPower(0);
                break;
        }
    }

    @Override
    public void stop() {

    }

    /* ===== Telemetry ===== */

    @Override
    public void addTelemetry() {
        telemetry.addLine("Intake");
        telemetry.addData("Mode", mode);
        telemetry.addData("Power", motor.getPower());
    }
}
