package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SwervePIDController {
    //TODO set PID constants
    public static double Kp = 0.0;
    public static double Ki = 0.0;
    public static double Kd = 0.0;

    //hardware
    private DcMotorEx encoder;
    private HardwareMap hwMap;
    private Telemetry telemetry;
    private ElapsedTime timer;  //a new timer is made, bc timer resets each loop

    //encoder constants
    private static final int TICKS_PER_REV = 8192;
    private static final double TICKS_PER_RAD = (TICKS_PER_REV) / (2.0 * Math.PI);

    //PID state
    private double prevError = 0.0;
    private double targetPos = 0.0;   // target heading in radians
    private double integralSum = 0.0;

    public SwervePIDController(String encoderName, HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.timer = new ElapsedTime();
        this.encoder = hwMap.get(DcMotorEx.class, encoderName);
    }

    private double ticksToRadians(int ticks) {
        return ticks / TICKS_PER_RAD;
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    public void setTargetHeading(double headingRadians) {
        targetPos = headingRadians;
    }

    public double update() {
        double currentHeading = ticksToRadians(encoder.getCurrentPosition());
        double error = angleWrap(targetPos - currentHeading);

        double dt = timer.seconds();
        double derivative = (error - prevError) / dt;
        integralSum += error * dt;

        double output = Kp * error + Ki * integralSum + Kd * derivative;

        prevError = error;
        timer.reset();

        return output;
    }
}