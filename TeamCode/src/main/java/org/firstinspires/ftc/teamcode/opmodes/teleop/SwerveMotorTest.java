package org.firstinspires.ftc.teamcode.opmodes.teleop;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
@TeleOp(name = "Swerve Motor Test", group = "Testing")
public class SwerveMotorTest extends LinearOpMode {
    public static double motor1pow = 0;
    public static double motor2pow = 0;
    public static double motor3pow = 0;
    public static double motor4pow = 0;
    public static boolean active = false;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        DcMotorEx motor3 = hardwareMap.get(DcMotorEx.class, "motor3");
        DcMotorEx motor4 = hardwareMap.get(DcMotorEx.class, "motor4");


        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (active) {
                    motor1.setPower(motor1pow);
                    motor2.setPower(motor2pow);
                    motor3.setPower(motor3pow);
                    motor4.setPower(motor4pow);
                } else {
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                    motor4.setPower(0);
                }

            }
        }
    }
}