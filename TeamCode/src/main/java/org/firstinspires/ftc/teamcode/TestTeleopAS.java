package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestTeleopAS", group = "")

public class TestTeleopAS extends LinearOpMode {
    private Blinker expansion_Hub_1;
    private Gyroscope imu;
    private DcMotor motor_drive_flAsDcMotor;
    private DcMotor motor_drive_frAsDcMotor;
    private DcMotor motor_drive_blAsDcMotor;
    private DcMotor motor_drive_brAsDcMotor;
    private DcMotor clarm;
    private Servo claw;
    boolean gamepadaAfter = false;
    boolean gamepadbAfter = false;

    @Override
    public void runOpMode() {
        motor_drive_flAsDcMotor = hardwareMap.dcMotor.get("motor_drive_flAsDcMotor");
        motor_drive_frAsDcMotor = hardwareMap.dcMotor.get("motor_drive_frAsDcMotor");
        motor_drive_blAsDcMotor = hardwareMap.dcMotor.get("motor_drive_blAsDcMotor");
        motor_drive_brAsDcMotor = hardwareMap.dcMotor.get("motor_drive_brAsDcMotor");
        clarm = hardwareMap.dcMotor.get("clarm");
        clarm.setDirection(DcMotor.Direction.FORWARD);
        clarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clarm.setPower(0);
        clarm.setTargetPosition(0);
        clarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        claw = hardwareMap.servo.get("claw");
        motor_drive_brAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_drive_frAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_drive_blAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_drive_flAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //clarm.setDirection(DcMotorSimple.Direction.FORWARD);
        // claw servo setup
        motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_blAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_brAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_blAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_brAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.update();
        waitForStart();

        waitForStart();
        while (opModeIsActive()) {
            motor_drive_flAsDcMotor.setPower(-gamepad1.left_stick_y - (-gamepad1.left_stick_x - gamepad1.right_stick_x));
            motor_drive_blAsDcMotor.setPower(gamepad1.left_stick_y - (-gamepad1.left_stick_x + gamepad1.right_stick_x));
            motor_drive_frAsDcMotor.setPower(-gamepad1.left_stick_y + (-gamepad1.left_stick_x - gamepad1.right_stick_x));
            motor_drive_brAsDcMotor.setPower(gamepad1.left_stick_y + (-gamepad1.left_stick_x + gamepad1.right_stick_x));

            if (!gamepad1.a && gamepadaAfter) {
                claw.setPosition(.17);
            }
            if (!gamepad1.b && gamepadbAfter) {
                claw.setPosition(.5);
            }
            gamepadaAfter = gamepad1.a;
            gamepadbAfter = gamepad1.b;


        }
    }
}