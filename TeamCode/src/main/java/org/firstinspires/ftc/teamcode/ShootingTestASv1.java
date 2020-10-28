package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ShootingTestASv1", group = "")

public class ShootingTestASv1 extends LinearOpMode{
    private Blinker expansion_Hub_1;
    private DcMotor LeftShooter;
    private DcMotor RightShooter;

    @Override
    public void runOpMode() {
        LeftShooter = hardwareMap.dcMotor.get("LeftShooter");
        RightShooter = hardwareMap.dcMotor.get("RightShooter");
        LeftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double Run;
        Run = 1;
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a){
                Run = -1;
            }
            if (gamepad1.b){
                Run = 1;
            }
            if (Run == 1) {
                LeftShooter.setPower(0);
                RightShooter.setPower(0);
            }

            if (Run == -1) {
                if (gamepad1.dpad_left){
                    LeftShooter.setPower(0.75);
                    RightShooter.setPower(-0.1);
                }
                else if (gamepad1.dpad_right){
                    LeftShooter.setPower(0.1);
                    RightShooter.setPower(-0.75);
                }
                else {
                    LeftShooter.setPower(-.8);
                    RightShooter.setPower(.6);
                    telemetry.addData("LeftMotorPosition", LeftShooter.getCurrentPosition());
                    telemetry.addData("RightMotorPosition", RightShooter.getCurrentPosition());
                    telemetry.update();
                }
            }
        }
    }
}