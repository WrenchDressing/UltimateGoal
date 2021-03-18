/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="PersonalityStateMachines", group="")
public class PersonalityStateMachines extends LinearOpMode {

    private enum State {
        MOVE_RIGHT,
        MOVE_LEFT,
        MOVE_FORWARD,
        MOVE_BACKWARDS,
        SHOOTING,
        INTAKE,
        IDLE,
    }

    private DcMotor motor1;
    private Servo servo1;
    private State CurrentState;

    @Override
    public void runOpMode() {

        switch (CurrentState) {
            case MOVE_RIGHT:
                if (gamepad1.right_stick_x > 0 || gamepad1.left_stick_x > 0) {
                    //eyeL turn to look right
                    //eyeR turn to look right
                    //light strip on right side turn on
                } else if (gamepad1.right_stick_x < 0 || gamepad1.left_stick_x < 0) {
                    CurrentState = State.MOVE_LEFT;
                } else if (gamepad1.left_stick_y > 0) {
                    CurrentState = State.MOVE_FORWARD;
                } else if (gamepad1.left_stick_y < 0) {
                    CurrentState = State.MOVE_BACKWARDS;
                } else if (RightLauncher.getPower < 0) {
                    CurrentState = State.SHOOTING;
                } else if (intake.getPower > 0) {
                    CurrentState = State.INTAKE;
                } else {
                    CurrentState = State.IDLE;
                }
                break;

            case MOVE_LEFT:
                if (gamepad1.right_stick_x < 0 || gamepad1.left_stick_x < 0) {
                    //eye servo1 turn to look left
                    //eye serv02 turn to look left
                    //light strip on left side turn on
                } else if (gamepad1.right_stick_x > 0 || gamepad1.left_stick_x > 0) {
                    CurrentState = State.MOVE_RIGHT;
                } else if (gamepad1.left_stick_y > 0) {
                    CurrentState = State.MOVE_FORWARD;
                } else if (gamepad1.left_stick_y < 0) {
                    CurrentState = State.MOVE_BACKWARDS;
                } else if (RightLauncher.getPower < 0) {
                    CurrentState = State.SHOOTING;
                } else if (intake.getPower > 0) {
                    CurrentState = State.INTAKE;
                } else {
                    CurrentState = State.IDLE;
                }
                break;

            case MOVE_FORWARD:
                if (gamepad1.left_stick_y > 0) {
                    //BrowR turns down
                    //BrowL turns down
                    //light strip on front turns on
                } else if (gamepad1.right_stick_x > 0 || gamepad1.left_stick_x > 0) {
                    CurrentState = State.MOVE_RIGHT;
                } else if (gamepad1.right_stick_x < 0 || gamepad1.left_stick_x < 0) {
                    CurrentState = State.MOVE_LEFT;
                } else if (gamepad1.left_stick_y < 0) {
                    CurrentState = State.MOVE_BACKWARDS;
                } else if (RightLauncher.getPower < 0) {
                    CurrentState = State.SHOOTING;
                } else if (intake.getPower > 0) {
                    CurrentState = State.INTAKE;
                } else {
                    CurrentState = State.IDLE;
                }
                break;

            case MOVE_BACKWARDS:
                if (gamepad1.left_stick_y < 0) {
                    //BrowR turns up
                    //BrowL turns up
                    //light strip on back turns on
                } else if (gamepad1.right_stick_x > 0 || gamepad1.left_stick_x > 0) {
                    CurrentState = State.MOVE_RIGHT;
                } else if (gamepad1.right_stick_x < 0 || gamepad1.left_stick_x < 0) {
                    CurrentState = State.MOVE_LEFT;
                } else if (gamepad1.left_stick_y > 0) {
                    CurrentState = State.MOVE_FORWARD;
                } else if (RightLauncher.getPower < 0) {
                    CurrentState = State.SHOOTING;
                } else if (intake.getPower > 0) {
                    CurrentState = State.INTAKE;
                } else {
                    CurrentState = State.IDLE;
                }
                break;

            case SHOOTING:
                if (RightLauncher.getPower < 0) {
                    //BrowR turn down
                    //BrowL turn down
                    //light strip flash
                } else if (gamepad1.right_stick_x > 0 || gamepad1.left_stick_x > 0) {
                    CurrentState = State.MOVE_RIGHT;
                } else if (gamepad1.right_stick_x < 0 || gamepad1.left_stick_x < 0) {
                    CurrentState = State.MOVE_LEFT;
                } else if (gamepad1.left_stick_y > 0) {
                    CurrentState = State.MOVE_FORWARD;
                } else if (gamepad1.left_stick_y < 0) {
                    CurrentState = State.MOVE_BACKWARDS;
                } else if (intake.getPower > 0) {
                    CurrentState = State.INTAKE;
                } else {
                    CurrentState = State.IDLE;
                }
                break;

            case INTAKE:
                if (intake.getPower > 0) {
                    //BrowR turn down
                    //BrowL turn down
                    //light strip flash
                } else if (gamepad1.right_stick_x > 0 || gamepad1.left_stick_x > 0) {
                    CurrentState = State.MOVE_RIGHT;
                } else if (gamepad1.right_stick_x < 0 || gamepad1.left_stick_x < 0) {
                    CurrentState = State.MOVE_LEFT;
                } else if (gamepad1.left_stick_y > 0) {
                    CurrentState = State.MOVE_FORWARD;
                } else if (gamepad1.left_stick_y < 0) {
                    CurrentState = State.MOVE_BACKWARDS;
                } else if (RightLauncher.getPower < 0) {
                    CurrentState = State.SHOOTING;
                } else {
                    CurrentState = State.IDLE;
                }
                break;

            case IDLE:
                if ((RightLauncher.getPower() = 0) && (intake.getPower() = 0) && (gamepad1.right_stick_x = 0) && (gamepad1.left_stick_y = 0) && (gamepad1.left_stick_x = 0)) {
                    //pattern on the light strip
                } else if (gamepad1.right_stick_x > 0 && gamepad1.left_stick_x > 0) {
                    CurrentState = State.MOVE_RIGHT;
                } else if (gamepad1.right_stick_x < 0 && gamepad1.left_stick_x < 0) {
                    CurrentState = State.MOVE_LEFT;
                } else if (gamepad1.left_stick_y > 0) {
                    CurrentState = State.MOVE_FORWARD;
                } else if (gamepad1.left_stick_y < 0) {
                    CurrentState = State.MOVE_BACKWARDS;
                } else if (RightLauncher.getPower < 0) {
                    CurrentState = State.SHOOTING;
                } else {
                    CurrentState = State.INTAKE;
                }
                break;

        }



        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y) {
                servo1.setPosition(0.25);
            }
            if (gamepad1.b) {
                servo1.setPosition(0.5);
            }
            if (gamepad1.a) {
                servo1.setPosition(0.75);
            }
            if (gamepad1.x) {
                servo1.setPosition(0);
            }
        }
    }
}
