package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.SwervePIDController;

@Config
public class SwerveModule
{
    private final DcMotorEx motor1;
    private final DcMotorEx motor2;
    private final DcMotorEx encoder;

    private final String motor1Name;
    private final String motor2Name;

    private final HardwareMap hwMap;
    private final Telemetry telemetry;
    private final ElapsedTime timer;

    private final String name;

    private final SwervePIDController headingPID;

    public static boolean telemActive = false;
    private boolean killPow = true;
    private double heading;
    private static final int TICKS_PER_REV = 8192;
    private static final double TICKS_PER_DEG = TICKS_PER_REV / 360.0;

    public SwerveModule(String podName, String motor1Name, boolean motor1Forward, String motor2Name, Boolean motor2Forward, String encoderName,
                        boolean encoderForward, HardwareMap hwMap, Telemetry telemetry, ElapsedTime timer)
    {

        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.timer = timer;
        this.name = podName;

        this.motor1 = hwMap.get(DcMotorEx.class, motor1Name);
        this.motor1Name = motor1Name;
        if(motor1Forward) {motor1.setDirection(DcMotorSimple.Direction.FORWARD);}
        else {motor1.setDirection(DcMotorSimple.Direction.REVERSE);}

        this.motor2 = hwMap.get(DcMotorEx.class, motor2Name);
        this.motor2Name = motor2Name;
        if(motor2Forward) {motor2.setDirection(DcMotorSimple.Direction.FORWARD);}
        else {motor2.setDirection(DcMotorSimple.Direction.REVERSE);}

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        this.encoder = hwMap.get(DcMotorEx.class, encoderName);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        headingPID = new SwervePIDController(encoderName, encoderForward, hwMap, telemetry);
    }

    private double ticksToDegrees(int ticks)
    {
        return (ticks / TICKS_PER_DEG);
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

    public void toggleKillPow(boolean input)
    {
        killPow = input;
    }

    public void toggleKillPow()
    {
        killPow = !killPow;
    }


    public boolean getkillPow()
    {
        return killPow;
    }

    public void toggleTelem()
    {
        telemActive = !telemActive;
    }

    public void toggleTelem(boolean input)
    {
        telemActive = input;
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

        // Wheel reversal if path > 110°
        //Was 90 deg but would flip when not necessary
        if (Math.abs(error) > 110)
        {
            driveSpeed *= -1; // reverse wheel
            headingDegrees = angleWrap(headingDegrees + 180.0); // rotate 180°
        }

        // Update PID target
        setTargetHeading(headingDegrees);

        // PID output for heading correction
        double turnPow = headingPID.update();

        // Differential motor mixing
        double motor1Power = turnPow + driveSpeed;
        double motor2Power = turnPow - driveSpeed;

        // Normalize if needed
        double max = Math.max(Math.abs(motor1Power), Math.abs(motor2Power));
        if (max > 1.0)
        {
            motor1Power /= max;
            motor2Power /= max;
        }

        // Apply powers
        if(killPow)
        {
            motor1.setPower(0);
            motor2.setPower(0);
        }
        else
        {
            motor1.setPower(motor1Power);
            motor2.setPower(motor2Power);
        }

        // Telemetry
        if (telemActive)
        {
            telemetry.addData(name+" Active", !killPow);
            telemetry.addData(name+" Target Speed", driveSpeed);
            telemetry.addData(name+" Target Heading", headingDegrees);
            telemetry.addData(name+" Real Heading", currentHeading);
            telemetry.addData(name+" Turn Pow", turnPow);
            telemetry.addData(name+" Speed Pow", driveSpeed);
            telemetry.addData(motor1Name+" Power", motor1Power);
            telemetry.addData(motor2Name+" Power", motor2Power);
        }
    }

    public void setPID(double p, double i, double d)
    {
        headingPID.setPID(p,i,d);
    }
}
