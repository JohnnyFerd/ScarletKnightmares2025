package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class outake extends Subsystem {

    /* ===== Hardware ===== */
    private final DcMotorEx leader;
    private final DcMotorEx follower;
    private final Telemetry telemetry;

    /* ===== Preset Velocities (RPM) ===== */
    public static int CloseShotVelo = 1680; // close shot RPM
    public static int MaxRPM = 5300;        // REV Core Hex max RPM (used for FarShot)

    /* ===== State ===== */
    private Mode mode = Mode.OFF;
    private int currentVelo = MaxRPM; // default FarShot

    private enum Mode { ON, OFF }

    public outake(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leader = hwMap.get(DcMotorEx.class, "launcherbot");
        follower = hwMap.get(DcMotorEx.class, "launchertop");

        leader.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        follower.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leader.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        follower.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Reverse one motor if needed for proper rotation
        follower.setDirection(DcMotorEx.Direction.REVERSE);
    }

    /* ===== Commands ===== */
    public void outakeOn() { mode = Mode.ON; }
    public void outakeOff() { stop(); }

    public void setFarShot() { currentVelo = MaxRPM; }    // Far shot uses max motor velocity
    public void setCloseShot() { currentVelo = CloseShotVelo; }

    /* ===== Update Loop ===== */
    @Override
    public void update() {
        double targetTicksPerSec = rpmToTicksPerSec(currentVelo);

        switch (mode) {
            case ON:
                leader.setVelocity(targetTicksPerSec);
                follower.setVelocity(targetTicksPerSec);
                break;

            case OFF:
            default:
                stop();
                break;
        }
    }

    /* ===== Stop Method ===== */
    @Override
    public void stop() {
        leader.setVelocity(0);
        follower.setVelocity(0);
        mode = Mode.OFF;
    }

    /* ===== Helpers ===== */
    private double rpmToTicksPerSec(double rpm) {
        return rpm * 28 / 60.0; // REV Core Hex = 28 ticks/rev
    }

    private double getRPM() {
        return leader.getVelocity() * 60.0 / 28;
    }

    /* ===== Telemetry ===== */
    @Override
    public void addTelemetry() {
        telemetry.addLine("Flywheel");
        telemetry.addData("Mode", mode);
        telemetry.addData("Target RPM", currentVelo);
        telemetry.addData("Actual RPM", getRPM());
    }
}
