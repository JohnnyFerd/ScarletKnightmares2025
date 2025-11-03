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

    private final Servo shooterServo1;
    private final Servo shooterServo2;
    private final Servo paddle1;
    private final Servo paddle2;

    public static double paddle1Down = .8;
    public static double paddle2Down = .15;
    public static double paddle1Up = .55;
    public static double paddle2Up = .4;

    public static double paddle1Pos = paddle1Down;
    public static double paddle2Pos = paddle2Down;


    //TODO tune motor PID for velocity
    public static double maxVelocity = 1700;
    public static double P = 100;
    public static double I = 10;
    public static double D = 10;
    public static double F = 10;

    private Shooter shooter;

    public static boolean shooterActive = false;
    public static double angle = 0.525;
    private final HardwareMap hwMap;
    private final Telemetry telemetry;
    private final ElapsedTime timer = new ElapsedTime();

    public Shooter(String shooter1Name, String shooter2Name, String shooterServo1, String shooterServo2, String paddle1, String paddle2, HardwareMap hwMap, Telemetry telemetry, ElapsedTime timer)
    {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

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

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Shooter(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot)
    {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        paddle1 = robot.paddle1;
        paddle2 = robot.paddle2;
        shooterServo1 = robot.shooterServo1;
        shooterServo2 = robot.shooterServo2;
        shooter1 = robot.shooter1;
        shooter2 = robot.shooter2;
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
        this.angle = angle;
    }


    public void setPaddle(double temp1, double temp2)
    {
        paddle1Pos = temp1;
        paddle2Pos = temp2;
    }

    public void togglePaddle()
    {
        if (paddle1.getPosition() == paddle1Down) {paddle1Pos = paddle1Up; paddle2Pos = paddle2Up;}
        else {paddle1Pos = paddle1Down; paddle2Pos = paddle2Down;}
    }

    @Override
    public void addTelemetry() {
        if (UseTelemetry.SHOOTER_TELEMETRY) {
            telemetry.addLine("Shooter Telemetry: ON");

            telemetry.addData("   SHOOTER1/SHOOTER2 Velocity", "%4.0f, %4.0f", shooter1.getVelocity(), shooter2.getVelocity());
            telemetry.addData("   Shooter Angle             ", "%4.5f", angle);
        }else {
            telemetry.addLine("Shooter Telemetry: OFF");
    }
        }

        //updates servos so position can be updated through dashboard
    @Override
    public void update()
    {
        if (shooterServo2.getPosition() != angle) {shooterServo1.setPosition(1-angle); shooterServo2.setPosition(angle);}
        if (paddle1.getPosition() != paddle1Pos) {paddle1.setPosition(paddle1Pos);}
        if (paddle2.getPosition() != paddle2Pos) {paddle2.setPosition(paddle2Pos);}
    }

    @Override
    public void stop()
    {
    }
}