package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Swerve Test", group = "Testing")
public class SwerveTest extends LinearOpMode {

    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;
    private ElapsedTime runtime = new ElapsedTime();

    public static double MOTOR_POWER = 0;
    public static double SERVO_ENCODER = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.update();

        DcMotorEx drive_motor = hardwareMap.get(DcMotorEx.class, "drive_motor");
        CRServo servo = hardwareMap.get(CRServo.class, "servo");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                if (gamepad1.left_stick_y > 0) {
                    drive_motor.setPower(gamepad1.left_stick_y);
                }else if (gamepad1.left_stick_y < 0) {
                    drive_motor.setPower(gamepad1.left_stick_y);
                }else {
                    drive_motor.setPower(MOTOR_POWER);
                }

                if (gamepad1.right_stick_x > 0) {
                    servo.setPower(gamepad1.right_stick_x);
                }else if (gamepad1.right_stick_x < 0) {
                    servo.setPower(gamepad1.right_stick_x);
                }else {
                    servo.setPower(SERVO_ENCODER);
                }
                
                telemetry.update();
            }
        }
    }
}
