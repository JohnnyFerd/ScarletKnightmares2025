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

    public static double paddle1Up = .3;
    public static double paddle2Up = .4;

    public static double paddle1Pos = .55;
    public static double paddle2Pos = .15;

    public static double angle = 0;
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
            shooter1.setPower(1);
            shooter2.setPower(1);
        }
        else {
            shooter1.setPower(0);
            shooter2.setPower(0);
        }

        if (paddle1.getPosition() != paddle1Pos) {paddle1.setPosition(paddle1Pos);paddle2.setPosition(paddle2Pos);}
        if (shooterServo1.getPosition() != angle) {shooterServo1.setPosition(angle); shooterServo2.setPosition(1 - angle);}

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
        if (paddle1Pos == paddle1Down) {paddle1Pos = paddle1Up; paddle2Pos = paddle2Up;}
        else {paddle1Pos = paddle1Down; paddle2Pos = paddle2Down;}
    }
}