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
    private final Servo headingServo;
    private final Servo paddle1;
    private final Servo paddle2;

    public static double shooter1Power = 1;
    public static double shooter2Power = 1;

    private final HardwareMap hwMap;
    private final Telemetry telemetry;
    private final ElapsedTime timer;
    public Shooter(String shooter1Name, boolean shooter1Forward, String shooter2Name, boolean shooter2Forward, String headingServo, String paddle1, String paddle2, HardwareMap hwMap, Telemetry telemetry, ElapsedTime timer)
    {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.timer = timer;

        this.shooter1 = hwMap.get(DcMotorEx.class, shooter1Name);
        this.shooter2 = hwMap.get(DcMotorEx.class, shooter2Name);

        this.headingServo = hwMap.get(Servo.class, headingServo);
        this.paddle1 = hwMap.get(Servo.class, paddle1);
        this.paddle2 = hwMap.get(Servo.class, paddle2);


        this.headingServo.setPosition(0);
        this.paddle1.setPosition(0);
        this.paddle2.setPosition(0);

        if (!shooter1Forward) {shooter1.setDirection(DcMotorSimple.Direction.FORWARD);} else {shooter1.setDirection(DcMotorSimple.Direction.REVERSE);}
        if (!shooter2Forward) {shooter2.setDirection(DcMotorSimple.Direction.REVERSE);} else {shooter2.setDirection(DcMotorSimple.Direction.FORWARD);}
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update(boolean shoot)
    {
        if (shoot)
        {
            shooter1.setPower(shooter1Power);
            shooter2.setPower(shooter2Power);
        }
        else {
            shooter1.setPower(0);
            shooter2.setPower(0);
        }
    }

    public void setAngle(double angle)
    {
        headingServo.setPosition(angle);
    }
}