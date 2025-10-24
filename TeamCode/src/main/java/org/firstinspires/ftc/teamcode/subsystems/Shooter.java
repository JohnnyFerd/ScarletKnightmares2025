package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Shooter {
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
    public static double pow = 1;
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

    }

    public void update(boolean shoot)
    {
        if (shoot)
        {
            shooter1.setPower(pow);
            shooter2.setPower(pow);
        }
        else {
            shooter1.setPower(0);
            shooter2.setPower(0);
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
}