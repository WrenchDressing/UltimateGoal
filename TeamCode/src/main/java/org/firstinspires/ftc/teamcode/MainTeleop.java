/*to do list
- convert dc motor to dc motor ex
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(name = "MAINteleop", group = "")

public class MainTeleop extends LinearOpMode {
    private static final String VUFORIA_KEY =
            "Ae2mEyz/////AAABmQBmoTE94ki5quwzTT/OlIIeOueUfjuHL/5k1VNWN943meU2RmiXCJ9eX3rUR/2CkwguvbBU45e1SzrbTAwz3ZzJXc7XN1ObKk/7yPHQeulWpyJgpeZx+EqmZW6VE6yG4mNI1mshKI7vOgOtYxqdR8Yf7YwBPd4Ruy3NVK01BwBl1F8V/ndY26skaSlnWqpibCR3XIvVG0LXHTdNn/ftZyAFmCedLgLi1UtNhr2eXZdr6ioikyRYEe7qsWZPlnwVn5DaQoTcgccZV4bR1/PEvDLn7jn1YNwSimTC8glK+5gnNpO+X7BiZa5LcqtYEpvk/QNQda0Fd+wHQDXA8ojeMUagawtkQGJvpPpz9c6p4fad";
    private static final float mmPerInch = 25.4f;
    private VuforiaLocalizer vuforia;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;
    // WebcamName webcamName = null;
    private BNO055IMU imu;
    private VuforiaCurrentGame vuforiaUltimateGoal;
    // Class Members
    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 90;
    private Blinker expansion_Hub_1;
    private DcMotor motor_drive_flAsDcMotor;
    private DcMotor motor_drive_frAsDcMotor;
    private DcMotor motor_drive_blAsDcMotor;
    private DcMotor motor_drive_brAsDcMotor;
    private DcMotor clarm;
    private Servo claw;
    private CRServo Conveyor;
    private Servo ramp;
    double Xposition;
    double Yposition;
    double TrueTrackSwitch;
    double IMUTrackSwitch;
    double Accelerate, Decelerate, XAccelerate, XDecelerate;
    double CurrentVal;
    double YSpeed;
    double lsPower, rsPower;
    double LastVal;
    double mXR, mYL, mXL, flScale, frScale, blScale, brScale;
    double CurrentX, CurrentY;
    double mFr, mFl, mBl, mBr;
    double YPositionReset, XPositionReset, XPositionP, YPositionP;
    double ClawVariable;
    boolean frOverload, flOverload, blOverload, brOverload;
    Orientation angles;
    boolean gamepadaAfter = false;
    boolean gamepada2After = false;
    boolean gamepad2lbafter = false;
    boolean gamepadbAfter = false;
    boolean lastGamepadX = false;
    boolean lastGamepadY = false;
    double lastClarm;
    double X, Y, XD, YD;
    private DcMotor intake;
    private DcMotor RightLauncher;
    private DcMotor LeftLauncher;
    boolean lastDPadUp = false;
    boolean lastDPadDown = false;
    double gamepadxpolar;
    boolean lastDPadRight = false;
    boolean LastDPadUp2 = false, LastDpadDown2 = false;
    boolean lastDPadLeft = false;
    boolean lastBumper = false;
    float CurrentHeading;
    double clarmvariable;
    private Servo camServo;
    double LastValue = 0.0;
    double PosDiffValue;
    double NegDiffValue;
    double FinalAngle = 0.0;
    double MaintainAngle = 0;
    ElapsedTime TimerA;


    private enum State {
        IDLE,
        FORWARD,
        BACKWARDS,
        STRAFE_RIGHT,
        STRAFE_LEFT
    }

    private State CurrentState;


    @Override
    public void runOpMode() {
        motor_drive_flAsDcMotor = hardwareMap.dcMotor.get("motor_drive_flAsDcMotor");
        motor_drive_frAsDcMotor = hardwareMap.dcMotor.get("motor_drive_frAsDcMotor");
        motor_drive_blAsDcMotor = hardwareMap.dcMotor.get("motor_drive_blAsDcMotor");
        motor_drive_brAsDcMotor = hardwareMap.dcMotor.get("motor_drive_brAsDcMotor");
        clarm = hardwareMap.dcMotor.get("clarm");
        clarm.setDirection(DcMotor.Direction.FORWARD);
        intake = hardwareMap.dcMotor.get("intake");
        RightLauncher = hardwareMap.dcMotor.get("RightLauncher");
        LeftLauncher = hardwareMap.dcMotor.get("LeftLauncher");
        Conveyor = hardwareMap.get(CRServo.class, "Conveyor");
        TimerA = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        ramp = hardwareMap.get(Servo.class, "ramp");
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //start of homing for clarm
        lastClarm = clarm.getCurrentPosition();
        clarm.setPower(-0.4);
        sleep(200);
        while (lastClarm != clarm.getCurrentPosition()) {
            lastClarm = clarm.getCurrentPosition();
            sleep(100);
        }
        clarm.setPower(0);
        clarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clarm.setPower(0);
        clarm.setTargetPosition(0);
        clarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //end of homing for clarm

        Initialization();


        ramp.setPosition(0.6);
        claw = hardwareMap.servo.get("claw");
        motor_drive_brAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_drive_frAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_drive_blAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_drive_flAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_blAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_brAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_blAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_brAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.update();
        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);


        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 5.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 9.0f;     // eg: Camera is ON the robot's center line
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, 90, 0, 90));


        /**  Let all the trackable listeners know where the phone is.  */

        targetVisible = false;

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:
        ClawVariable = 1;
        clarmvariable = 0;

        CurrentState = State.IDLE;

        waitForStart();


        //Set this to ZERO if you want to disable testing mode ^^^^^
        clarmvariable = 1;
        //targetsUltimateGoal.activate();
        //camServo.setPosition(.5);
        while (opModeIsActive()) {
            switch (CurrentState) {
                case FORWARD:
                    if (gamepad1.left_stick_y < 0) {
                        claw.setPosition(0.8);
                        telemetry.addData("Forward", CurrentState);
                        telemetry.update();
                    } else if (gamepad1.left_stick_y > 0) {
                        CurrentState = State.BACKWARDS;
                    } else if (gamepad1.left_stick_x > 0) {
                        CurrentState = State.STRAFE_RIGHT;
                    } else if (gamepad1.left_stick_x < 0) {
                        CurrentState = State.STRAFE_LEFT;
                    } else {
                        CurrentState = State.IDLE;
                    }
                    break;

                case BACKWARDS:
                    if (gamepad1.left_stick_y > 0) {
                        claw.setPosition(0.7);
                        telemetry.addData("Backwards", CurrentState);
                        telemetry.update();
                    } else if (gamepad1.left_stick_y < 0) {
                        CurrentState = State.FORWARD;
                    } else if (gamepad1.left_stick_x > 0) {
                        CurrentState = State.STRAFE_RIGHT;
                    } else if (gamepad1.left_stick_x < 0) {
                        CurrentState = State.STRAFE_LEFT;
                    } else {
                        CurrentState = State.IDLE;
                    }
                    break;

                case STRAFE_LEFT:
                    if (gamepad1.left_stick_x < 0) {
                        claw.setPosition(0.9);
                        telemetry.addData("Strafe Left", CurrentState);
                        telemetry.update();
                    } else if (gamepad1.left_stick_y > 0) {
                        CurrentState = State.BACKWARDS;
                    } else if (gamepad1.left_stick_x > 0) {
                        CurrentState = State.STRAFE_RIGHT;
                    } else if (gamepad1.left_stick_y < 0) {
                        CurrentState = State.FORWARD;
                    } else {
                        CurrentState = State.IDLE;
                    }
                    break;

                case STRAFE_RIGHT:
                    if (gamepad1.left_stick_x > 0) {
                        claw.setPosition(1);
                        telemetry.addData("Strafe Right", CurrentState);
                        telemetry.update();
                    } else if (gamepad1.left_stick_y > 0) {
                        CurrentState = State.BACKWARDS;
                    } else if (gamepad1.left_stick_x < 0) {
                        CurrentState = State.STRAFE_LEFT;
                    } else if (gamepad1.left_stick_y < 0) {
                        CurrentState = State.FORWARD;
                    } else {
                        CurrentState = State.IDLE;
                    }
                    break;

                case IDLE:
                    if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0) {
                        claw.setPosition(0);
                        telemetry.addData("Idle", CurrentState);
                        telemetry.update();
                    } else if (gamepad1.left_stick_y > 0) {
                        CurrentState = State.BACKWARDS;
                    } else if (gamepad1.left_stick_x < 0) {
                        CurrentState = State.STRAFE_LEFT;
                    } else if (gamepad1.left_stick_x > 0) {
                        CurrentState = State.STRAFE_RIGHT;
                    } else {
                        CurrentState = State.FORWARD;
                    }
            }

            ContinuedIMU();
            if (gamepad1.left_stick_x < 0) {
                gamepadxpolar = -1;
            } else if (gamepad1.left_stick_x >= 0) {
                gamepadxpolar = 1;
            }
            MecanumFunction(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, FinalAngle);
            /*motor_drive_flAsDcMotor.setPower(-gamepad1.left_stick_y + (gamepadxpolar * (gamepad1.left_stick_x * gamepad1.left_stick_x)) + gamepad1.right_stick_x);
            motor_drive_blAsDcMotor.setPower(gamepad1.left_stick_y + (gamepadxpolar * (gamepad1.left_stick_x * gamepad1.left_stick_x)) - gamepad1.right_stick_x);
            motor_drive_frAsDcMotor.setPower(-gamepad1.left_stick_y + (gamepadxpolar * (gamepad1.left_stick_x * gamepad1.left_stick_x)) - gamepad1.right_stick_x);
            motor_drive_brAsDcMotor.setPower(gamepad1.left_stick_y + (gamepadxpolar * (gamepad1.left_stick_x * gamepad1.left_stick_x)) + gamepad1.right_stick_x);*/
            CurrentX = 0;
            CurrentY = 0;
            /*
            if (gamepad1.a && !gamepada2After) {
                wallTargetTracking(vuforia, allTrackables, 90, -2.5, 41.8, 0, 10, 2, 1, 3, false, 0);
            }
            */

            LastDpadDown2 = gamepad1.dpad_down;
            if (gamepad2.b && !gamepadbAfter) {
                ClawVariable = ClawVariable * -1;
            }
                /*if (ClawVariable > 0) {
                    claw.setPosition(0);
                } else if (ClawVariable < 0) {
                    claw.setPosition(1);
                }*/

            gamepadaAfter = gamepad2.a;
            gamepadbAfter = gamepad2.b;
            gamepada2After = gamepad1.a;


            if (gamepad2.dpad_up && !lastDPadUp) {
                ramp.setPosition(0.0);

                RightLauncher.setPower(-0.37);
                LeftLauncher.setPower(0.79);

                intake.setPower(0);
                Conveyor.setPower(0);
            }
            if (gamepad2.left_bumper && !gamepad2lbafter) {
                ramp.setPosition(0);

                RightLauncher.setPower(-0.31);
                LeftLauncher.setPower(0.745);

                intake.setPower(0);
                Conveyor.setPower(0);

            }
            gamepad2.left_bumper = gamepad2lbafter;
            if (gamepad2.dpad_down && !lastDPadDown) {
                ramp.setPosition(0.6);
                RightLauncher.setPower(0);
                LeftLauncher.setPower(0);
                Conveyor.setPower(0);
            }
            lastDPadUp = gamepad2.dpad_up;
            lastDPadDown = gamepad2.dpad_down;


            //  lastDPadRight = gamepad2.dpad_right;
            //taking in rings
            if (gamepad2.dpad_right) {//&& !lastDPadRight) {
                intake.setPower(-1);
                Conveyor.setPower(-1);
                //spitting out rings
            } else if (gamepad2.dpad_left) {//&& !lastDPadLeft) {
                intake.setPower(0.8);
                Conveyor.setPower(0.8);
            } else {
                intake.setPower(0);
                Conveyor.setPower(0);
            }


            //  lastDPadLeft = gamepad2.dpad_left;

            if ((!gamepad2.dpad_right) && (!gamepad2.dpad_left)) {
                intake.setPower(0);
                if (ramp.getPosition() > 0.3) { // Check here to make sure we don't step on the other Conveyor.setPower commands below
                    Conveyor.setPower(0);
                }
            }


            if (gamepad2.right_bumper && !lastBumper) {
                clarmvariable = -clarmvariable;
                if (clarmvariable > 0) {
                    clarm.setTargetPosition(990);
                    clarm.setPower(0.4);
                }
                if (clarmvariable < 0) {
                    clarm.setTargetPosition(400);
                    clarm.setPower(0.4);
                }
            }

            lastBumper = gamepad2.right_bumper;


        }
    }

    private void BulkCaching() {
        mFr = motor_drive_frAsDcMotor.getCurrentPosition();
        mFl = motor_drive_flAsDcMotor.getCurrentPosition();
        mBr = motor_drive_brAsDcMotor.getCurrentPosition();
        mBl = motor_drive_blAsDcMotor.getCurrentPosition();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        CurrentHeading = angles.firstAngle;
        telemetry.addData("IMU", CurrentHeading);
        telemetry.addData("LeftShooterPower", LeftLauncher.getPower());
        telemetry.addData("RightShooterPower", RightLauncher.getPower());
        telemetry.update();
        if (targetVisible) {


        }
    }

    private void Initialization() {
        BNO055IMU.Parameters imuParameters;

        Acceleration gravity;
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters); //  <<<====  THIS LINE WAS MISSING
        // webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        clarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clarm.setTargetPosition(0);
        clarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        CurrentHeading = angles.firstAngle;

        motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_blAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_brAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      /*  VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);
        camServo = hardwareMap.get(Servo.class, "camServo");


        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        //Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, 90, 0, 90));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(webcamName, robotFromCamera);
        }

//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // initTfod();
        //webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
    }
*/
    }

    private void PowerShot() {
        GoTo(0, 20, .3, 0, 14, 0, true, true, 0.012, true);
        AngularAdjustment(0, 0.28);
    }

    private void AngularAdjustment(double targetangle, double IMUgain) {
        ElapsedTime TimerD = new ElapsedTime();
        TimerD.reset();
        while ((TimerD.milliseconds() < 350)) {

            // ANDREW: Consider the use of BulkCaching here since it is expensive to read all the motor positions if all you need is the imu angle
            BulkCaching();
            if (((-targetangle + CurrentHeading) >= 0.07 && (-targetangle + CurrentHeading) <= -0.07)) {
                TimerD.reset();
            }
            MecanumFunction(0, 0, IMUgain * (-targetangle + CurrentHeading), 0);
        }
        MecanumFunction(0, 0, 0, 0);
    }

    private void Shoot(double RP, double LP, double Timer, boolean Stop) {
        ElapsedTime ShootTimer;
        ShootTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        ShootTimer.reset();
        while (ShootTimer.seconds() <= Timer) {
            LeftLauncher.setPower(LP);
            RightLauncher.setPower(RP);
            Conveyor.setPower(1);
        }

        if (Stop == true) {
            LeftLauncher.setPower(0);
            RightLauncher.setPower(0);
        }
        Conveyor.setPower(0);

    }

    private void ContinuedIMU() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if ((LastValue - angles.firstAngle) > 180) {
            PosDiffValue = 360 - (LastValue - angles.firstAngle);
            FinalAngle += PosDiffValue;
        } else if ((LastValue - angles.firstAngle) < -180) {
            NegDiffValue = 360 + (LastValue - angles.firstAngle);
            FinalAngle -= NegDiffValue;
        } else {
            FinalAngle += LastValue - angles.firstAngle;
        }
        LastValue = angles.firstAngle;
        telemetry.addData("FinalAngle", FinalAngle);
        telemetry.addData("LastValue", LastValue);
        telemetry.addData("CurrentValue", angles.firstAngle);
        telemetry.update();
    }

    private void DistanceTracker() {
        if (Yposition < 0) {
            YPositionP = -1;
        } else {
            YPositionP = 1;
        }
        if (Xposition < 0) {
            XPositionP = -1;
        } else {
            XPositionP = 1;
        }
        Yposition = (((((motor_drive_blAsDcMotor.getCurrentPosition() + motor_drive_brAsDcMotor.getCurrentPosition() + motor_drive_flAsDcMotor.getCurrentPosition() + motor_drive_frAsDcMotor.getCurrentPosition()) * 0.25) * 0.00208333333) * 15.75) - YPositionReset);//((((motor_drive_brAsDcMotor.getCurrentPosition() + motor_drive_blAsDcMotor.getCurrentPosition() + motor_drive_flAsDcMotor.getCurrentPosition() + motor_drive_frAsDcMotor.getCurrentPosition()) / 4) / 480) * 12.566);
        Xposition = ((((motor_drive_flAsDcMotor.getCurrentPosition() - motor_drive_blAsDcMotor.getCurrentPosition()) * 0.00208333333) * 11.5 * .5) - (XPositionReset));

    }

    private void ResetTracker() {
        XPositionReset = Xposition;
        YPositionReset = Yposition;
    }

    private void GoTo(double XDistance, double YDistance, double Speed, double MaintainAngle,
                      double AccelDistance, double DecelDistance, boolean Accel, boolean Decel, double IMUGain,
                      boolean Reset) {

        Accelerate = Math.abs(Yposition) * 0.7;
        XAccelerate = Math.abs(Xposition) * 0.989;
        Decelerate = ((Math.abs(YDistance) - Math.abs(Yposition)) * 0.4);
        XDecelerate = ((Math.abs(XDistance) - Math.abs(Xposition)) * 0.6);
        ResetTracker();
        DistanceTracker();


        /*
        if (Accel) {
            motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_drive_blAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_drive_brAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_drive_blAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_drive_brAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Xposition = 0;
            Yposition = 0;
        }
*/
        X = 1;
        Y = 1;
        XD = 1;
        YD = 1;
        if (!(XDistance == 0)) {

            while (Math.abs(Xposition) <= Math.abs(XDistance) || Math.abs(Yposition) <= Math.abs(YDistance)) {
                // telemetry.addData("Speed", Math.max(Math.min(Speed * ((Math.abs(Yposition) * 0.006) / Accelerate) - 0/*(Decelerate * (1 / (Math.abs(YDistance) - Math.abs(Yposition))))*/, 0.0001), Speed * 0.707106781 * Math.abs((YDistance / XDistance)) * Y));
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                CurrentHeading = angles.firstAngle;

                DistanceTracker();

                if (Math.abs(Xposition) >= Math.abs(XDistance)) {
                    if (Decel) {
                        X = 0;
                    }
                }
                if (Math.abs(Yposition) >= Math.abs(YDistance)) {
                    if (Decel) {
                        Y = 0;
                    }
                }
                if (XDistance < 0) {
                    XD = -1;
                }
                if (YDistance < 0) {
                    YD = -1;
                }
                if (Accel && ((Math.abs(Xposition) <= Math.abs((AccelDistance * 0.7071067)) || Math.abs(Yposition) <= Math.abs(AccelDistance)))) {

                    Accelerate = 1;
                    XAccelerate = 1;
                    if (Math.abs(Xposition) <= 8 || Math.abs(Yposition) <= 13) {
                        telemetry.addData("Accel?", 1);
                        telemetry.update();
                    }
                } else {
                    Accelerate = Math.abs(Yposition) * 0.6;
                    XAccelerate = Math.abs(Xposition) * 0.7;
                }
                if (Decel && ((Math.abs(YDistance) - Math.abs(Yposition) < 13 || Math.abs(XDistance) - Math.abs(Xposition) < 8))) {
                    if (Math.abs(YDistance) - Math.abs(Yposition) < 13 || Math.abs(XDistance) - Math.abs(Xposition) < 8) {
                        //  telemetry.addData("Decel?", 1);
                        Decelerate = 1;
                        XDecelerate = 1;
                        // telemetry.update();

                    }
                } else {
                    Decelerate = ((Math.abs(YDistance) - Math.abs(Yposition)) * 0.4);
                    XDecelerate = ((Math.abs(XDistance) - Math.abs(Xposition)) * 1);
                }
                telemetry.addData("fart", (Math.min(Math.max(Speed * ((Math.abs(Xposition) * 1) / XAccelerate) * (((Math.abs(XDistance) - Math.abs(Xposition)) * 0.6) / XDecelerate), 0.49), Speed) * X * XD));
                //    telemetry.addData("fart", (((Math.abs(XDistance) - Math.abs(Xposition)) * 0.6) / XDecelerate));
                telemetry.update();
                MecanumFunction(Math.min(Math.max(Speed * ((Math.abs(Yposition) * 0.6) / Accelerate) * (((Math.abs(YDistance) - Math.abs(Yposition)) * 0.4) / Decelerate), 0.05), YSpeed) * Y * YD, Math.min(Math.max(Speed * ((Math.abs(Xposition) * 1) / XAccelerate) * (((Math.abs(XDistance) - Math.abs(Xposition)) * 0.7) / XDecelerate), 0.49), Speed) * X * XD, (-MaintainAngle + CurrentHeading) * IMUGain, 0);


                YSpeed = Math.abs(Speed * 0.707106781 * Math.abs((YDistance / XDistance)) * Y);
            }

            if (Decel) {
                MecanumFunction(0, 0, 0, 0);
            }

        } else if (XDistance == 0) {

            while (Math.abs(Yposition) <= Math.abs(YDistance)) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                CurrentHeading = angles.firstAngle;
                DistanceTracker();

                if (Math.abs(Yposition) >= Math.abs(YDistance)) {
                    if (Decel) {
                        Y = 0;
                    }
                }
                if (YDistance < 0) {
                    YD = -1;
                }
                if (Accel && Math.abs(Yposition) <= Math.abs(AccelDistance)) {

                    Accelerate = 1;
                    if (Math.abs(Yposition) <= 13) {
                        telemetry.addData("Accel?", 1);
                        telemetry.update();
                    }
                } else {
                    Accelerate = Math.abs(Yposition) * 0.6;
                }
                if (Decel && ((Math.abs(YDistance) - Math.abs(Yposition) < 13))) {
                    if (Math.abs(YDistance) - Math.abs(Yposition) < 13) {
                        //  telemetry.addData("Decel?", 1);
                        Decelerate = 1;
                        // telemetry.update();

                    }
                } else {
                    Decelerate = ((Math.abs(YDistance) - Math.abs(Yposition)) * 0.4);
                }

                MecanumFunction(Math.min(Math.max(Speed * ((Math.abs(Yposition) * 0.6) / Accelerate) * (((Math.abs(YDistance) - Math.abs(Yposition)) * 0.4) / Decelerate), 0.05), Speed) * Y * YD, 0, (-MaintainAngle + CurrentHeading) * IMUGain, 0);

            }

            if (Decel) {
                MecanumFunction(0, 0, 0, 0);
            }


        }
    }


    private void MecanumFunction(double YL, double XL, double XR, double ContinuosIMU) {
        double Correction;
        double NXL;
        double NYL;
        double NXR = 0;
        double IMUGain = -0.025;

        if (gamepad1.right_stick_x != 0) {
            MaintainAngle = ContinuosIMU;
            TimerA.reset();
        }
        if (TimerA.seconds() >= .2) {
            NXR = (ContinuosIMU - MaintainAngle) * IMUGain;
        } else {
            MaintainAngle = ContinuosIMU;
        }
        XR += NXR;
        telemetry.addData("NewXR", NXR);
        telemetry.addData("IMU", ContinuosIMU);
        telemetry.update();

        flScale = (-YL - (-XL - XR));
        blScale = (YL - (-XL + XR));
        frScale = (-YL + XL - XR);
        brScale = (YL + XL + XR);

        /*motor_drive_flAsDcMotor.setPower(-gamepad1.left_stick_y + (gamepadxpolar * (gamepad1.left_stick_x * gamepad1.left_stick_x)) + gamepad1.right_stick_x);
            motor_drive_blAsDcMotor.setPower(gamepad1.left_stick_y + (gamepadxpolar * (gamepad1.left_stick_x * gamepad1.left_stick_x)) - gamepad1.right_stick_x);
            motor_drive_frAsDcMotor.setPower(-gamepad1.left_stick_y + (gamepadxpolar * (gamepad1.left_stick_x * gamepad1.left_stick_x)) - gamepad1.right_stick_x);
            motor_drive_brAsDcMotor.setPower(gamepad1.left_stick_y + (gamepadxpolar * (gamepad1.left_stick_x * gamepad1.left_stick_x)) + gamepad1.right_stick_x);*/

        if ((-YL - (-XL - XR)) > 1 || (-YL - (-XL - XR)) < -1) {
            flOverload = true;
        } else {
            flOverload = false;
        }
        if ((YL - (-XL + XR)) > 1 || (YL - (-XL + XR)) < -1) {
            blOverload = true;
        } else {
            blOverload = false;
        }
        if ((-YL + XL - XR) > 1 || (-YL + XL - XR) < -1) {
            frOverload = true;
        } else {
            frOverload = false;
        }
        if ((YL + (-XL + XR)) > 1 || (YL + (-XL + XR)) < -1) {
            brOverload = true;
        } else {
            brOverload = false;
        }
        if (frOverload == true || flOverload == true || blOverload == true || brOverload == true) {
            if (flScale > frScale && flScale > brScale && flScale > blScale) {
                mYL = YL / Math.abs(flScale);
                mXL = XL / Math.abs(flScale);
                mXR = XR / Math.abs(flScale);
            } else if (frScale > flScale && frScale > brScale && frScale > blScale) {
                mYL = YL / Math.abs(frScale);
                mXL = XL / Math.abs(frScale);
                mXR = XR / Math.abs(frScale);
            } else if (blScale > frScale && blScale > brScale && blScale > flScale) {
                mYL = YL / Math.abs(blScale);
                mXL = XL / Math.abs(blScale);
                mXR = XR / Math.abs(blScale);
            } else if (brScale > frScale && brScale > flScale && brScale > blScale) {
                mYL = YL / Math.abs(brScale);
                mXL = XL / Math.abs(brScale);
                mXR = XR / Math.abs(brScale);
            }

        }
        if (frOverload == false && flOverload == false && brOverload == false && blOverload == false) {
            motor_drive_flAsDcMotor.setPower((-YL - (-XL - XR)));
            motor_drive_blAsDcMotor.setPower(-(YL - (-XL + XR)));
            motor_drive_frAsDcMotor.setPower((-YL - XL - XR));
            motor_drive_brAsDcMotor.setPower(-(YL - XL + XR));
        } else {
            motor_drive_flAsDcMotor.setPower((-mYL - (-mXL - mXR)));
            motor_drive_blAsDcMotor.setPower(-(mYL - (-mXL + mXR)));
            motor_drive_frAsDcMotor.setPower((-mYL - mXL - mXR));
            motor_drive_brAsDcMotor.setPower(-(mYL - mXL + mXR));
        }


    }

   /* private void wallTargetTracking(VuforiaLocalizer vufor, Iterable<? extends VuforiaTrackable> allTrackables,
                                    int camDir, double xTarget, double yTarget, double yawTarget, double linTol, double angleTol, double pGain,
                                    long timeOut, boolean TrueTracking, double TargetAngle) {

              I am a function that tracks a wall target using Vuforia using parameters passed on to me
                camDir - Direction that my webcam is pointed.  -1 is to the left, 0 is straight ahead, 1 is to the right
                xTarget - X coordinate target value
                yTarget - Y coord target value
                yawTarget - yaw angle target value
                linTol - XY distance tolerance before deciding we are at target location
                angleTol - angular tolerance before deciding we are at the target location
                pGain - Overall Proportional gain for motor commands based on error
                timeOut - How long to wait before exiting function

        if (TrueTracking == true) {
            TrueTrackSwitch = 1;
            IMUTrackSwitch = 0;
        } else if (TrueTracking == false) {
            IMUTrackSwitch = 1;
            TrueTrackSwitch = 0;
        }
        boolean exitFlag = false;
        ElapsedTime exitTimer = new ElapsedTime();
        exitTimer.reset();
        LastVal = CurrentVal = 175;
        //   (!exitFlag && (exitTimer.time() <= timeOut)) ||
        while ((!(yTarget - CurrentY <= 0.75 && yTarget - CurrentY >= -0.75 && xTarget - CurrentX <= 0.75 && xTarget - CurrentX >= -0.75 && (-TargetAngle + CurrentHeading) <= 2 && (-TargetAngle + CurrentHeading) >= -2))) {
            if (isStopRequested()) {
                break;
            }
            BulkCaching();
            // express the rotation of the robot in degrees.
            // check all the trackable targets to see which one (if any) is visible.
            telemetry.addData("YOffset", 18.1 - CurrentY);
            telemetry.addData("XOffset", 1.9 - CurrentX);
            telemetry.update();
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                } else {
                    targetVisible = false;
                }
            }


            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                VectorF translation = lastLocation.getTranslation();
                CurrentY = translation.get(1) / mmPerInch;
                CurrentX = translation.get(0) / mmPerInch;
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                CurrentHeading = angles.firstAngle;

                //telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                //        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

                CurrentVal = rotation.thirdAngle;
                        //                   telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                        if (LastVal - CurrentVal > 180){
                            CurrentVal += 360;
                        }
                        else if(LastVal - CurrentVal < -180){
                            CurrentVal -= 360;
                        }
                        if (TrueTrackSwitch == 1){
                            telemetry.addData("Switch on!",1);
                        }
                        else if (TrueTrackSwitch == 0) {
                            telemetry.addData("Switch Off!", -TargetAngle + CurrentHeading);
                        }

              //  telemetry.update();

//                        Heading = (180 + (180 - rotation.thirdAngle));
              //if (camDir == 90) {
            //MecanumFunction(1 * (-0.013 * (xTarget - CurrentX)), (-1 * (0.021 * (yTarget - CurrentY))), TrueTrackSwitch * (-0.0075 * (yawTarget - (CurrentVal))) + (-1 * IMUTrackSwitch * -0.02 * (-TargetAngle + CurrentHeading)));
                }
                //else if (camDir == 90){
                //    MecanumFunction(0 * (-0.02 * (40 - CurrentX)), (0 * (-0.02 * (-3 - CurrentY))), TrueTrackSwitch * (0.0008 * (yawTarget - (180 + Zrot) + (IMUTrackSwitch * 0.0008 * (-TargetAngle + CurrentHeading)))));
                //}
                //else {
                //    MecanumFunction(0 * (-0.02 * (40 - CurrentX)), (0 * (-0.02 * (-3 - CurrentY))), TrueTrackSwitch * (0.0008 * (yawTarget - (180 + Zrot) + (IMUTrackSwitch * 0.0008 * (-TargetAngle + CurrentHeading)))));

                //}
            } else {
                telemetry.addData("Visible Target", "none");

                MecanumFunction(0, 0, 0);
                break;
            }
//                    telemetry.update();
            LastVal = CurrentVal;

        }
        */
}



