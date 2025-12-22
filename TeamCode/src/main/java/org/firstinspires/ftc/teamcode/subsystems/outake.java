package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class outake extends Subsystem {

    /* ===== Hardware ===== */
    private final DcMotorEx leader;   // encoder motor
    private final DcMotorEx follower; // no encoder
    private final Telemetry telemetry;

    /* ===== Tunables ===== */
    public static double TARGET_RPM = 3000;
    public static double TICKS_PER_REV = 28; // change to match motor

    /* ===== State ===== */
    private Mode mode = Mode.OFF;

    private enum Mode {
        ON,
        REVERSE,
        OFF
    }

    public outake(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leader = hwMap.get(DcMotorEx.class, "launcherbot");
        follower = hwMap.get(DcMotorEx.class, "launchertop");

        leader.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        follower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        follower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


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
            case ON: {
                double velocity = rpmToTicksPerSec(TARGET_RPM);
                leader.setVelocity(velocity);
                follower.setPower(leader.getPower());
                break;
            }

            case REVERSE: {
                double velocity = rpmToTicksPerSec(-TARGET_RPM);
                leader.setVelocity(velocity);
                follower.setPower(leader.getPower());
                break;
            }

            case OFF:
            default:
                leader.setPower(0);
                follower.setPower(0);
                break;
        }
    }

    @Override
    public void stop() {
        leader.setPower(0);
        follower.setPower(0);
    }

    /* ===== Helpers ===== */

    private double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    private double getRPM() {
        return leader.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    /* ===== Telemetry ===== */

    @Override
    public void addTelemetry() {
        telemetry.addLine("Flywheel");
        telemetry.addData("Mode", mode);
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Actual RPM", getRPM());
        telemetry.addData("Leader Power", leader.getPower());
    }
}
