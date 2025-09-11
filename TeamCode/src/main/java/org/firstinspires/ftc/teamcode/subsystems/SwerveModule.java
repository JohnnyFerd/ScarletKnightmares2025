package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SwerveModule
{
    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private DcMotorEx encoder; // incremental encoder on one of the motors

    private final HardwareMap hwMap;
    private final Telemetry telemetry;
    private final ElapsedTime timer;

    private final SwervePIDController headingPID;
    private static final int TICKS_PER_REV = 8192;
    private static final double TICKS_PER_RAD = TICKS_PER_REV / (2.0 * Math.PI);

    public SwerveModule(String motor1Name, String motor2Name, String encoderName,
                        HardwareMap hwMap, Telemetry telemetry, ElapsedTime timer)
    {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.timer = timer;

        this.motor1 = hwMap.get(DcMotorEx.class, motor1Name);
        this.motor2 = hwMap.get(DcMotorEx.class, motor2Name);
        this.encoder = hwMap.get(DcMotorEx.class, encoderName);

        headingPID = new SwervePIDController(encoderName, hwMap, telemetry);
    }

    private double ticksToRadians(int ticks)
    {
        return ticks / TICKS_PER_RAD;
    }

    private double angleWrap(double angle)
    {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    public void setTargetHeading(double headingRadians)
    {
        headingPID.setTargetHeading(headingRadians);
    }

    /**
     * Sets desired wheel speed and heading for incremental encoder.
     * Wheel reversal implemented for shortest path.
     * @param driveSpeed - forward/backward command (-1 to 1)
     * @param headingRadians - desired heading in radians
     */
    public void setDesiredState(double driveSpeed, double headingRadians)
    {
        // Current wheel angle from incremental encoder
        double currentHeading = ticksToRadians(encoder.getCurrentPosition());

        // Shortest path error
        double error = angleWrap(headingRadians - currentHeading);

        // Wheel reversal if path > 90°
        if (Math.abs(error) > Math.PI / 2)
        {
            driveSpeed *= -1; // reverse wheel
            headingRadians = angleWrap(headingRadians + Math.PI); // rotate 180°
        }

        // Update PID target
        setTargetHeading(headingRadians);

        // PID output for heading correction
        double angleOutput = headingPID.update();

        // Differential motor mixing
        double motor1Power = driveSpeed + angleOutput;
        double motor2Power = driveSpeed - angleOutput;

        // Normalize if needed
        double max = Math.max(Math.abs(motor1Power), Math.abs(motor2Power));
        if (max > 1.0)
        {
            motor1Power /= max;
            motor2Power /= max;
        }

        // Apply powers
        motor1.setPower(motor1Power);
        motor2.setPower(motor2Power);

        // Telemetry
        telemetry.addData("Motor1 Power", motor1Power);
        telemetry.addData("Motor2 Power", motor2Power);
        telemetry.addData("Current Heading", currentHeading);
        telemetry.addData("Target Heading", headingRadians);

        telemetry.update();
    }
}
