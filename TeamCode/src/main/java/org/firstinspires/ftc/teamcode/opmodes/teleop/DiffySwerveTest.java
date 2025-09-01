package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.settings.RobotSettings;

@Disabled
@TeleOp(name = "Diffy Swerve Test", group = "Testing")
public class DiffySwerveTest extends LinearOpMode {

    public static double MOTOR1_POWER = 0;
    public static double MOTOR2_Power = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", RobotSettings.SUPER_TIME.toString());
        telemetry.update();

        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, "motor2");

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                if (gamepad1.left_stick_y > 0) {
                    motor1.setPower(gamepad1.left_stick_y);
                }else if (gamepad1.left_stick_y < 0) {
                    motor1.setPower(gamepad1.left_stick_y);
                }else {
                    motor1.setPower(MOTOR1_POWER);
                }

                if (gamepad1.right_stick_x > 0) {
                    motor2.setPower(gamepad1.right_stick_x);
                }else if (gamepad1.right_stick_x < 0) {
                    motor2.setPower(gamepad1.right_stick_x);
                }else {
                    motor2.setPower(MOTOR2_Power);
                }

                double encoderPos = (motor1.getCurrentPosition());
                double degrees = (encoderPos % 8192.0) / 8192.0 * 360.0;

                telemetry.addData("Encoder Pos: ", encoderPos);
                telemetry.addData("Degrees", degrees);

                telemetry.update();
            }
        }
    }
}
