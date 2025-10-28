package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.UseTelemetry;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
public class Shooter extends Subsystem {
    private final DcMotorEx shooter1;
    private final DcMotorEx shooter2;


    public static boolean shooter1Forward = false;
    public static boolean shooter2Forward = false;

    private final Servo shooterServo1;
    private final Servo shooterServo2;
    private final Servo paddle1;
    private final Servo paddle2;

    public static double paddle1Down = .55;
    public static double paddle2Down = .15;
    public static double paddle1Up = .25;
    public static double paddle2Up = .45;

    //TODO tune motor PID for velocity
    public static double maxVelocity = 6000;
    public static double P = 10;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;

    private Shooter shooter;


    public static double angle = 0.5;
    private final HardwareMap hwMap;
    private final Telemetry telemetry;
    private final ElapsedTime timer;

    public Shooter(String shooter1Name, String shooter2Name, String shooterServo1, String shooterServo2, String paddle1, String paddle2, HardwareMap hwMap, Telemetry telemetry, ElapsedTime timer)
    {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.timer = timer;

        this.shooter1 = hwMap.get(DcMotorEx.class, shooter1Name);
        this.shooter2 = hwMap.get(DcMotorEx.class, shooter2Name);

        this.shooterServo1 = hwMap.get(Servo.class, shooterServo1);
        this.shooterServo2 = hwMap.get(Servo.class, shooterServo2);

        this.paddle1 = hwMap.get(Servo.class, paddle1);
        this.paddle2 = hwMap.get(Servo.class, paddle2);


        this.shooterServo1.setPosition(angle);
        this.shooterServo2.setPosition(1-angle);
        this.paddle1.setPosition(paddle1Down);
        this.paddle2.setPosition(paddle2Down);

        if (shooter1Forward) {shooter1.setDirection(DcMotorSimple.Direction.FORWARD);} else {shooter1.setDirection(DcMotorSimple.Direction.REVERSE);}
        if (shooter2Forward) {shooter2.setDirection(DcMotorSimple.Direction.REVERSE);} else {shooter2.setDirection(DcMotorSimple.Direction.FORWARD);}
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        PIDFCoefficients pidf = new PIDFCoefficients(P,I,D,F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }


    //Sets velocity of motors for shooter based off of percent of max speed
    public void setVelocity(double temp)
    {
        if (Math.abs(temp) > 1){
            shooter1.setVelocity(maxVelocity);
            shooter2.setVelocity(maxVelocity);
        }
        else {
            shooter1.setVelocity(temp * maxVelocity);
            shooter2.setVelocity(temp * maxVelocity);
        }
    }

    public void setAngle(double angle)
    {
        shooterServo1.setPosition(angle); shooterServo2.setPosition(1 - angle);
    }

    public void setPaddle(double temp1, double temp2)
    {
        paddle1.setPosition(temp1);
        paddle2.setPosition(temp2);
    }

    public void togglePaddle()
    {
        if (paddle1.getPosition() == paddle1Down) {paddle1.setPosition(paddle1Up); paddle2.setPosition(paddle2Up);}
        else {paddle1.setPosition(paddle1Down); paddle2.setPosition(paddle2Down);}
    }

    @Override
    public void addTelemetry() {
        if (UseTelemetry.DRIVETRAIN_TELEMETRY) {
            telemetry.addLine("Shooter Telemetry: ON");

            telemetry.addData("   SHOOTER1/SHOOTER2 Velocity", "%4.0f, %4.0f", shooter1.getVelocity(), shooter2.getVelocity());
            telemetry.addData("   Shooter Angle             ", "%4.5f", angle);
        }else {
            telemetry.addLine("Shooter Telemetry: OFF");
    }
        }

    @Override
    public void update()
    {
        setAngle(angle);
    }

    @Override
    public void stop()
    {

    }
}