package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.UseTelemetry;

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
    public static double paddle1UpFar = .6;
    public static double paddle2UpFar = .35;
    public static double paddle1UpClose = .4;
    public static double paddle2UpClose = .45;
    public static double paddle1UpFarLast = .575;
    public static double paddle2UpFarLast = .375;
    public static int FarShotVelo = 1750;
    public static int CloseShotVelo = 1325;

    public static double paddle1Pos = paddle1Down;
    public static double paddle2Pos = paddle2Down;

    public static double P = 100;
    public static double I = 10;
    public static double D = 10;
    public static double F = 10;

    private double velocity = 0;

    public static boolean shooterActive = false;
    public static double angle = .6;

    private final HardwareMap hwMap;
    private final Telemetry telemetry;
    private final ElapsedTime timer = new ElapsedTime();

    public Shooter(String shooter1Name, String shooter2Name, String shooterServo1, String shooterServo2, String paddle1, String paddle2, HardwareMap hwMap, Telemetry telemetry, ElapsedTime timer) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        this.shooter1 = hwMap.get(DcMotorEx.class, shooter1Name);
        this.shooter2 = hwMap.get(DcMotorEx.class, shooter2Name);

        this.shooterServo1 = hwMap.get(Servo.class, shooterServo1);
        this.shooterServo2 = hwMap.get(Servo.class, shooterServo2);

        this.paddle1 = hwMap.get(Servo.class, paddle1);
        this.paddle2 = hwMap.get(Servo.class, paddle2);

        this.shooterServo1.setPosition(angle);
        this.shooterServo2.setPosition(1 - angle);
        this.paddle1.setPosition(paddle1Down);
        this.paddle2.setPosition(paddle2Down);
    }

    public Shooter(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        paddle1 = robot.paddle1;
        paddle2 = robot.paddle2;
        shooterServo1 = robot.shooterServo1;
        shooterServo2 = robot.shooterServo2;
        shooter1 = robot.shooter1;
        shooter2 = robot.shooter2;
    }

    public void setVelocity(double temp) {
        if (Math.abs(temp) > 2000) {temp = 2000;}

        shooter1.setVelocity(temp);
        shooter2.setVelocity(temp);
        velocity = temp;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    public void setPaddle(double temp1, double temp2) {
        paddle1Pos = temp1;
        paddle2Pos = temp2;
    }

    // Combined paddleUp and paddleUpClose logic
    public void paddleUp() {
        if (velocity == CloseShotVelo) {
            paddle1Pos = paddle1UpClose;
            paddle2Pos = paddle2UpClose;
        } else {
            paddle1Pos = paddle1UpFar;
            paddle2Pos = paddle2UpFar;
        }
    }

    public void paddleDown() {
        paddle1Pos = paddle1Down;
        paddle2Pos = paddle2Down;
    }

    // Now switches to Close “last” positions when last velocity = CloseShotVelo
    public void paddleUpLast() {
        if (velocity == CloseShotVelo) {
            paddle1Pos = paddle1UpClose;
            paddle2Pos = paddle2UpClose;
        } else {
            paddle1Pos = paddle1UpFarLast;
            paddle2Pos = paddle2UpFarLast;
        }
    }

    public double getVelocity()
    {
        return velocity;
    }

    @Override
    public void addTelemetry() {
        if (UseTelemetry.SHOOTER_TELEMETRY) {
            telemetry.addLine("Shooter Telemetry: ON");
            telemetry.addData("   SHOOTER1/SHOOTER2 Velocity", "%4.0f, %4.0f", shooter1.getVelocity(), shooter2.getVelocity());
            telemetry.addData("   Shooter Angle", "%4.5f", angle);
            telemetry.addData("   Last Velocity", velocity);
        } else {
            telemetry.addLine("Shooter Telemetry: OFF");
        }
    }

    @Override
    public void update() {
        if (shooterServo2.getPosition() != angle) {
            shooterServo1.setPosition(1 - angle);
            shooterServo2.setPosition(angle);
        }
        if (paddle1.getPosition() != paddle1Pos) {
            paddle1.setPosition(paddle1Pos);
        }
        if (paddle2.getPosition() != paddle2Pos) {
            paddle2.setPosition(paddle2Pos);
        }
    }

    @Override
    public void stop() {}
}
