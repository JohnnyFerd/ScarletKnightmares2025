package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.SwervePIDController;

public class SwerveModule
{
    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private DcMotorEx encoder;

    private double heading;

    private final HardwareMap hwMap;
    private final Telemetry telemetry;
    private final ElapsedTime timer;

    private final SwervePIDController headingPID;

    private boolean killPow = true;
    private static final int TICKS_PER_REV = 8192;
    private static final double TICKS_PER_DEG = TICKS_PER_REV / 360.0; // convert ticks to degrees

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

    private double ticksToDegrees(int ticks)
    {
        return ticks / TICKS_PER_DEG;
    }

    private double angleWrap(double angle)
    {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    public void setTargetHeading(double headingDegrees)
    {
        heading = headingDegrees;
        headingPID.setTargetHeading(headingDegrees);
    }

    public double getTargetHeading()
    {
        return heading;
    }

    public void toggleKillPow()
    {
        killPow = !killPow;
    }

    public boolean getkillPow()
    {
        return killPow;
    }

    /**
     * Sets desired wheel speed and heading
     * @param driveSpeed - forward/backward command (-1 to 1)
     * @param headingDegrees - desired heading in degrees
     */
    public void update(double driveSpeed, double headingDegrees)
    {
        // Current wheel angle from incremental encoder
        double currentHeading = ticksToDegrees(encoder.getCurrentPosition());

        // Shortest path error
        double error = angleWrap(headingDegrees - currentHeading);

        // Wheel reversal if path > 90°
        if (Math.abs(error) > 90.0)
        {
            driveSpeed *= -1; // reverse wheel
            headingDegrees = angleWrap(headingDegrees + 180.0); // rotate 180°
        }

        // Update PID target
        setTargetHeading(headingDegrees);

        // PID output for heading correction
        double turnPow = headingPID.update();

        // Differential motor mixing
        double motor1Power = driveSpeed + turnPow;
        double motor2Power = driveSpeed - turnPow;

        // Normalize if needed
        double max = Math.max(Math.abs(motor1Power), Math.abs(motor2Power));
        if (max > 1.0)
        {
            motor1Power /= max;
            motor2Power /= max;
        }

        // Apply powers
        if (!killPow)
        {
            motor1.setPower(motor1Power);
            motor2.setPower(motor2Power);
        }

        // Telemetry
        telemetry.addData("Elapsed time", timer.toString());
        telemetry.addData("Active", !killPow);
        telemetry.addData("Target Speed", driveSpeed);
        telemetry.addData("Target Heading", headingDegrees);
        telemetry.addData("Real Heading", currentHeading);
        telemetry.addData("Motor1 Power", motor1Power);
        telemetry.addData("Motor2 Power", motor2Power);
    }
}
