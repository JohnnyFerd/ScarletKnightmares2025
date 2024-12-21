package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;

@Disabled
@TeleOp(name = "Motor and Servo Test", group = "Testing")
public class TestingOpmode extends LinearOpMode {

    private HardwareMap hwMap;
//    private Telemetry telemetry;
    private int counter1 = 0, counter2 = 0, counter3 = 0;
    private boolean motorsOn = false, servosOn = false;

    // MOTORS
    public static DcMotorEx[] motors = new DcMotorEx[8];
    public static double[] motorPowers = {0, 0, 0, 0, 0.1, 0.1, 0.1, 0.1};

    // SERVOS
    public static Servo[] servos = new Servo[12];
    public static DcMotorSimple[] crservos = new DcMotorSimple[6];
    public static double[] servoPositions1 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    public static double[] servoPosition2 = {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0};

    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        hwMap = hardwareMap;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", RobotSettings.SUPER_TIME.toString());
        telemetry.update();

        initHardware();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad2.copy(gamepad2);

                telemetry.addLine("X = Toggle Motor Break Behavior");
                telemetry.addLine("Y = Reverse Motor Direction");
                telemetry.addLine("A = Reverse Servo Direction");
                telemetry.addLine("Dpad UP = Turn Motors On/Off");
                telemetry.addLine("Dpad DOWN = Turn Servos to Position/0");

                if (currentGamepad1.x && !previousGamepad1.x) {
                    counter1++;
                    if (counter1 % 2 == 0) {
                        for (DcMotorEx motor : motors) {
                            if (motor != null) {
                                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            }
                        }
                    }else {
                        for (DcMotorEx motor : motors) {
                            if (motor != null) {
                                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                            }
                        }
                    }
                }
                if (currentGamepad1.y && !previousGamepad1.y) {
                    counter2++;
                    if (counter2 % 2 == 0) {
                        for (DcMotorEx motor : motors) {
                            if (motor != null) {
                                motor.setDirection(DcMotor.Direction.FORWARD);
                            }
                        }
                    }else {
                        for (DcMotorEx motor : motors) {
                            if (motor != null) {
                                motor.setDirection(DcMotor.Direction.REVERSE);
                            }
                        }
                    }
                }
                if (currentGamepad1.a && !previousGamepad1.a) {
                    counter3++;
                    if (counter3 % 2 == 0) {
                        for (Servo servo : servos) {
                            if (servo != null) {
                                servo.setDirection(Servo.Direction.FORWARD);
                            }
                        }
                    }else {
                        for (Servo servo : servos) {
                            if (servo != null) {
                                servo.setDirection(Servo.Direction.REVERSE);
                            }
                        }
                    }
                }
                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    motorsOn = !motorsOn;
                }
                if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    servosOn = !servosOn;
                }

                if (motorsOn) {
                    for (int i = 0; i < 8; i++) {
                        if (motors[i] != null) {
                            motors[i].setPower(motorPowers[i]);
                        }
                    }
                }else if (!motorsOn) {
                    for (int i = 0; i < 8; i++) {
                        if (motors[i] != null) {
                            motors[i].setPower(0);
                        }
                    }
                }

                if (!servosOn) {
                    for (int i = 0; i < 12; i++) {
                        if (servos[i] != null) {
                            servos[i].setPosition(servoPositions1[i]);
                        }
                    }
                    for (int i = 0; i < 6; i++) {
                        if (crservos[i] != null) {
                            crservos[i].setPower(servoPositions1[i]);
                        }
                    }
                }else if (servosOn) {
                    for (int i = 0; i < 12; i++) {
                        if (servos[i] != null) {
                            servos[i].setPosition(servoPosition2[i]);
                        }
                    }
                    for (int i = 0; i < 6; i++) {
                        if (crservos[i] != null) {
                            crservos[i].setPower(servoPosition2[i]);
                        }
                    }
                }


                if (counter1 % 2 == 0) {
                    telemetry.addLine("Motor Break Behavior: BREAK");
                }else if (counter1 % 2 == 1){
                    telemetry.addLine("Motor Break Behavior: FLOAT");
                }
                if (counter2 % 2 == 0) {
                    telemetry.addLine("Motor Direction: FORWARD");
                }else if (counter2 % 2 == 1) {
                    telemetry.addLine("Motor Direction: REVERSE");
                }
                if (counter3 % 2 == 0) {
                    telemetry.addLine("Servo Direction: FORWARD");
                }else if (counter3 % 2 == 1) {
                    telemetry.addLine("Servo Direction: REVERSE");
                }
                telemetry.addLine("----------MOTORS----------");
                for (int i = 0; i < 8; i++) {
                    if (motors[i] != null) {
                        telemetry.addData("Motor " + i + " Power: ", motors[i].getPower());
                    }
                }
                telemetry.addLine("----------SERVOS----------");
                for (int i = 0; i < 12; i++) {
                    if (servos[i] != null) {
                        telemetry.addData("Servo " + i + " Position: ", servos[i].getPosition());
                    }
                }

                telemetry.update();

            }
        }
    }

    public void initHardware() {
//        motors[0] = hwMap.get(DcMotorEx.class, "motor1");
//        motors[1] = hwMap.get(DcMotorEx.class, "motor2");
//        motors[2] = hwMap.get(DcMotorEx.class, "motor3");
//        motors[3] = hwMap.get(DcMotorEx.class, "motor4");
        motors[4] = hwMap.get(DcMotorEx.class, "motor5");
        motors[5] = hwMap.get(DcMotorEx.class, "motor6");
//        motors[6] = hwMap.get(DcMotorEx.class, "motor7");
//        motors[7] = hwMap.get(DcMotorEx.class, "motor8");

        for (DcMotorEx motor : motors) {
            if (motor != null) {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }

//        servos[0] = hwMap.get(Servo.class, "servo1");
//        crservos[0] = hwMap.get(DcMotorSimple.class, "servo1");

        servos[1] = hwMap.get(Servo.class, "servo2");
//        crservos[1] = hwMap.get(DcMotorSimple.class, "servo2");

//        servos[2] = hwMap.get(Servo.class, "servo1");
//        crservos[2] = hwMap.get(DcMotorSimple.class, "servo3");

//        servos[3] = hwMap.get(Servo.class, "servo1");
//        crservos[3] = hwMap.get(DcMotorSimple.class, "servo4");

//        servos[4] = hwMap.get(Servo.class, "servo1");
//        servos[5] = hwMap.get(Servo.class, "servo1");
//        servos[6] = hwMap.get(Servo.class, "servo1");
//        servos[7] = hwMap.get(Servo.class, "servo1");
//        servos[8] = hwMap.get(Servo.class, "servo1");
//        servos[9] = hwMap.get(Servo.class, "servo1");
//        servos[10] = hwMap.get(Servo.class, "servo1");
//        servos[11] = hwMap.get(Servo.class, "servo1");

        for (Servo servo : servos) {
            if (servo != null) {
                servo.setPosition(0);
            }
        }
    }

}
