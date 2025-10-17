package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class SwervePIDController {
    //TODO set PID constants
    private double Kp;
    private double Ki;
    private double Kd;

    //hardware
    private DcMotorEx encoder;
    private HardwareMap hwMap;
    private Telemetry telemetry;
    private ElapsedTime timer;
    private boolean encoderForward;

    //encoder constants
    private static final int TICKS_PER_REV = 8192;
    private static final double TICKS_PER_DEG = TICKS_PER_REV / 360.0;

    //PID state
    private double prevError = 0.0;
    private double targetPos = 0.0;   // target heading in degrees
    private double integralSum = 0.0;

    public SwervePIDController(String encoderName, boolean encoderForward, HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.timer = new ElapsedTime(); //a new timer is made, bc timer resets each loop
        this.encoder = hwMap.get(DcMotorEx.class, encoderName);
        this.encoderForward = encoderForward;
    }

    private double ticksToDegrees(int ticks) {
        return ticks / TICKS_PER_DEG;
    }

    private double angleWrap(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    public void setTargetHeading(double headingDegrees) {
        targetPos = headingDegrees;
    }

    public double update() {
        double currentHeading = ticksToDegrees(encoder.getCurrentPosition());
        if(!encoderForward){currentHeading *= -1.0;}
            double error = angleWrap(targetPos - currentHeading);

            double dt = timer.seconds();
            double derivative = (error - prevError) / dt;
            integralSum += error * dt;

            double output = Kp * error + Ki * integralSum + Kd * derivative;

            prevError = error;
            timer.reset();

            return output;
        }

        public void setPID (double kP, double kI, double kD)
        {
            this.Kp = kP;
            this.Ki = kI;
            this.Kd = kD;
        }
    }
