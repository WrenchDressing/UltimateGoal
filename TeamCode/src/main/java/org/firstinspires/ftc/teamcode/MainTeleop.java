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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
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
    double AccelConstant, AccelConstant2;
    double DecelConstant, DecelConstant2;
    double YL, XL;
    private Blinker expansion_Hub_1;
    private Servo leftbrow, lefteye, rightbrow, righteye;
    private DcMotorEx motor_drive_flAsDcMotor;
    private DcMotorEx motor_drive_frAsDcMotor;
    private DcMotorEx motor_drive_blAsDcMotor;
    private DcMotorEx motor_drive_brAsDcMotor;
    public static final double NEW_P = 20;
    public static final double NEW_I = 15;
    public static final double NEW_D = 0;
    public static final double NEW_F = 5;
    private Servo pusher;
    private ColorSensor rangeSensor_REV_ColorRangeSensor;
    NormalizedRGBA normalizedColors;
    int color;
    float hue;
    private Servo claw;
    private CRServo Conveyor;
    private DcMotorEx ramp;
    double Xposition;
    double Yposition;
    double TrueTrackSwitch;
    double IMUTrackSwitch;
    double Accelerate, Decelerate, XAccelerate, XDecelerate;
    double CurrentVal;
    double YSpeed;
    double lsPower, rsPower;
    double LastVal;
    double z;
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
    boolean gamepad2YAfter = false;
    boolean lastGamepadX = false;
    boolean lastDpadRight = false;
    boolean lastGamepadY = false;
    private Servo clarm;
    double lastClarm;
    double X, Y, XD, YD;
    private DcMotor intake;
    private DcMotor RightLauncher;
    private DcMotor LeftLauncher;
    boolean lastDPadUp = false;
    boolean lastDPadDown = false;
    double gamepadxpolar;
    boolean FullMag = false;
    boolean LastDPadUp2 = false, LastDpadDown2 = false;
    boolean lastDPadLeft = false;
    boolean lastBumper = false;
    float CurrentHeading;
    double clarmvariable;
    private Servo camServo;
    double currentrampvariable;
    double LastValue = 0.0;
    double PosDiffValue;
    double n = 1;
    double NegDiffValue;
    double FinalAngle = 0.0;
    double MaintainAngle = 0;
    ElapsedTime TimerA;
    ElapsedTime PusherTimer;
    boolean VPTimer = false;

    private enum State {
        //personality states
        MOVE_RIGHT,
        MOVE_LEFT,
        MOVE_FORWARD,
        MOVE_BACKWARDS,
        SHOOT,
        INTAKE,
        PERSONALITY_IDLE,
        MAD,
        //shoot and intake states
        DISC_COLLECTION,
        RAMP_TO_SHOOT,
        INTAKE_IDLE,
        //BREAK,
        POWERSHOT_RAMP,
        SHOOT_2,
    }

    private State PersonalityCurrentState;
    private State IntakeCurrentState;
    private State IntakeAndShootCurrentState;


    @Override
    public void runOpMode() {
        motor_drive_flAsDcMotor = hardwareMap.get(DcMotorEx.class, "motor_drive_flAsDcMotor");
        motor_drive_frAsDcMotor = hardwareMap.get(DcMotorEx.class, "motor_drive_frAsDcMotor");
        motor_drive_blAsDcMotor = hardwareMap.get(DcMotorEx.class, "motor_drive_blAsDcMotor");
        motor_drive_brAsDcMotor = hardwareMap.get(DcMotorEx.class, "motor_drive_brAsDcMotor");
        intake = hardwareMap.dcMotor.get("intake");
        righteye = hardwareMap.get(Servo.class, "righteye");
        lefteye = hardwareMap.get(Servo.class, "lefteye");
        clarm = hardwareMap.get(Servo.class, "clarm");
        leftbrow = hardwareMap.get(Servo.class, "leftbrow");
        rightbrow = hardwareMap.get(Servo.class, "rightbrow");
        pusher = hardwareMap.get(Servo.class, "pusher");
        RightLauncher = hardwareMap.dcMotor.get("RightLauncher");
        LeftLauncher = hardwareMap.dcMotor.get("LeftLauncher");
        Conveyor = hardwareMap.get(CRServo.class, "Conveyor");
        TimerA = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        PusherTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        ramp = hardwareMap.get(DcMotorEx.class, "ramp");
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        rangeSensor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "rangeSensor");

        //start of homing for clarm


        //end of homing for clarm

        Initialization();


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
        motor_drive_brAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_drive_blAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_drive_flAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_drive_frAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motor_drive_flAsDcMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        motor_drive_frAsDcMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        motor_drive_blAsDcMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        motor_drive_brAsDcMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        PIDFCoefficients rampPID = new PIDFCoefficients(12, 8f, 0, 0);
        ramp.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, rampPID);
        ramp.setPositionPIDFCoefficients(8);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ((NormalizedColorSensor) rangeSensor_REV_ColorRangeSensor).setGain(2);
        normalizedColors = ((NormalizedColorSensor) rangeSensor_REV_ColorRangeSensor).getNormalizedColors();
        color = normalizedColors.toColor();
        hue = JavaUtil.colorToHue(color);

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


        ClawVariable = 1;
        clarmvariable = 0;
        boolean gamepad2xafter = false;
        double m;
        m = 1;
        PersonalityCurrentState = State.PERSONALITY_IDLE;
        //IntakeCurrentState = State.INTAKE_IDLE;
        IntakeAndShootCurrentState = State.INTAKE_IDLE;
        currentrampvariable = 100;
        ramp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ramp.setPower(-0.2);
        sleep(50);
        while (currentrampvariable - ramp.getCurrentPosition() > 0 || currentrampvariable - ramp.getCurrentPosition() < 0) {
            ramp.setPower(-0.2);
            sleep(15);
            currentrampvariable = ramp.getCurrentPosition();
            if (isStopRequested()) {
                break;
            }
        }
        ramp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ramp.setTargetPosition(0);
        ramp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pusher.setPosition(0);
        boolean gamepad2a = false;
        double v = 1;
        boolean gamepad2rightbumperafter = false;

        waitForStart();

        //Set this to ZERO if you want to disable testing mode ^^^^^
        clarmvariable = 1;

        //targetsUltimateGoal.activate();
        //camServo.setPosition(.5);
        z = 1;
        boolean gamepad1Yafter = false;
        while (opModeIsActive()) {
            PersonalityStateMachine();
            //IntakeStateMachine();
            IntakeAndShoot();
            ContinuedIMU();
            if (gamepad1.left_stick_x < 0) {
                gamepadxpolar = -1;
            } else if (gamepad1.left_stick_x >= 0) {
                gamepadxpolar = 1;
            }
            if (gamepad1.left_trigger > 0.3) {
                MecanumFunction(-gamepad1.left_stick_y * 0.5 / 2, gamepad1.left_stick_x * 0.5 / 2, gamepad1.right_stick_x * 0.5 / 2);
            } else {
                MecanumFunction(-gamepad1.left_stick_y / 2, gamepad1.left_stick_x / 2, gamepad1.right_stick_x / 2);
            }
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
            if (gamepad2.dpad_right) {
                RightLauncher.setPower(0);
                LeftLauncher.setPower(0);
            }
            if (gamepad2.left_trigger > 0.3) {
                if (ramp.getCurrentPosition() > 300) {
                    pusher.setPosition(0.98);
                }
            }
            if (gamepad1.x) {
                pusher.setPosition(0.41);
            }
            if (gamepad1.y) {
                pusher.setPosition(0.775);
            }
            if (gamepad1.b) {
                pusher.setPosition(0.98);
            }
            gamepad2rightbumperafter = gamepad2.right_bumper;
            if (gamepad2.dpad_left) {
                n *= -1;
            }
            if (gamepad2.a && !gamepad2a) {
                v *= -1;
            }

            gamepad2a = gamepad2.a;

            if (v > 0) {
                if (gamepad2.x && !gamepad2xafter) {
                    clarmvariable = clarmvariable * -1;
                }
                if (clarmvariable > 0 && clarmvariable < 5) {
                    clarm.setPosition(0.12);
                } else if (clarmvariable > -5 && clarmvariable < 0) {
                    clarm.setPosition(0.55);
                }
            } else {
                clarm.setPosition(0);
            }


            gamepad2xafter = gamepad2.x;
            LastDpadDown2 = gamepad1.dpad_down;

            if (gamepad2.b && !gamepadbAfter) {
                ClawVariable = ClawVariable * -1;
            }
            if (ClawVariable > 0) {
                claw.setPosition(0);
            } else if (ClawVariable < 0) {
                claw.setPosition(1);
            }

            gamepadaAfter = gamepad2.a;
            gamepadbAfter = gamepad2.b;
            gamepada2After = gamepad1.a;


            /*if (gamepad2.dpad_up && !lastDPadUp) {
                RightLauncher.setPower(-0.2);
                LeftLauncher.setPower(0.2);
                ramp.setTargetPosition(816);
                ramp.setPower(0.6);
                sleep(500);
                RightLauncher.setPower(.8);
                LeftLauncher.setPower(-0.4);

                intake.setPower(0);
                Conveyor.setPower(0);
            }*/
/*
            if (gamepad2.left_bumper && !gamepad2lbafter) {
                ramp.setTargetPosition(0);
                ramp.setPower(-0.5);
                RightLauncher.setPower(-0.2);
                LeftLauncher.setPower(0.2);
                intake.setPower(1);


            }
            gamepad2.left_bumper = gamepad2lbafter;
            if (gamepad2.dpad_down && !lastDPadDown && pusher.getPosition() == 0) {
                ramp.setTargetPosition(0);
                ramp.setPower(-0.5);
                RightLauncher.setPower(0);
                LeftLauncher.setPower(0);
            }
            */

            lastDPadUp = gamepad2.dpad_up;
            lastDPadDown = gamepad2.dpad_down;


            // lastDPadRight = gamepad2.dpad_right;
            //taking in rings
            /*if (gamepad2.dpad_right && !lastDpadRight) {//&& !lastDPadRight) {
                intake.setPower(-1);
              pusher.setPosition(0.9);
                //spitting out rings
            } else if (gamepad2.dpad_left&& !lastDPadLeft) {//&& !lastDPadLeft) {
               // intake.setPower(0.8);
                pusher.setPosition(0);
            } else {

            }*/


            lastDPadLeft = gamepad2.dpad_left;


            //  lastDPadLeft = gamepad2.dpad_left;


            lastBumper = gamepad2.right_bumper;


        }
    }

    private void SpeedCalculator(double XDistance, double YDistance, double Speed, double AccelDistance, double DecelDistance, boolean Accel, boolean Decel) {
        if (Math.abs(Xposition) >= Math.abs(XDistance)) {
            if (Decel) {
                X = 0;
            }
        }
        if (XDistance == 0) {
            X = 0;
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
        if (XDistance == 0) {
            if (Accel && ((Math.abs(Yposition) <= Math.abs(AccelDistance)))) {
                YL = Math.min(Math.max(AccelConstant * (Yposition * Yposition), 0.025), YSpeed) * Y * YD;
                XL = Math.min(Math.max(AccelConstant2 * (Xposition * Xposition), 0.025), Speed) * X * XD;
            } else if (Decel && ((Math.abs(YDistance) - Math.abs(Yposition) <= Math.abs(DecelDistance)))) {
                YL = Math.min(Math.max(0.025 + (DecelConstant * Math.abs((YDistance - Yposition))), 0.025), YSpeed) * Y * YD;
                XL = Math.min(Math.max((0.025 + (DecelConstant2 * Math.abs(((XDistance - Xposition))))), 0.025), Speed) * X * XD;
            } else {
                YL = YSpeed * YD;
                XL = Speed * XD;
            }
        } else {
            if (Accel && ((Math.abs(Xposition) <= Math.abs((AccelDistance * 0.7071067)) || Math.abs(Yposition) <= Math.abs(AccelDistance)))) {
                YL = Math.min(Math.max(AccelConstant * (Yposition * Yposition), 0.025), YSpeed) * Y * YD;
                XL = Math.min(Math.max(AccelConstant2 * (Xposition * Xposition), 0.025), Speed) * X * XD;
                telemetry.addData("Accel?", 1);
                telemetry.update();
            } else if (Decel && ((Math.abs(YDistance) - Math.abs(Yposition) <= Math.abs(DecelDistance) || Math.abs(XDistance) - Math.abs(Xposition) <= Math.abs(DecelDistance * 0.7071067)))) {
                YL = Math.min(Math.max(0.025 + (DecelConstant * (YDistance - Yposition)), 0.025), YSpeed) * Y * YD;
                XL = Math.min(Math.max((0.025 + (DecelConstant2 * ((XDistance - Xposition)))), 0.025), Speed) * X * XD;
                telemetry.addData("Decel?", 1);
                telemetry.update();
            } else {
                YL = YSpeed;
                XL = Speed;
                telemetry.addData("Neither?", 1);
                telemetry.update();
            }
        }


    }

    private void GoTov2(double XDistance, double YDistance, double Speed, double MaintainAngle, double AccelDistance, double DecelDistance, boolean Accel, boolean Decel, double IMUGain, boolean Reset) {
        if (Reset) {
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
        } else {
            DistanceTracker();
            ResetTracker();
        }
        XPositionReset = Xposition;
        YPositionReset = Yposition;
        DistanceTracker();
        YSpeed = Math.abs(Speed * 0.7071067);
        AccelConstant = (Speed / AccelDistance);
        AccelConstant2 = (Speed / (AccelDistance * 0.7071067));
        DecelConstant = (((YSpeed - 0.025) / DecelDistance));
        DecelConstant2 = (((Speed - 0.025) / (DecelDistance * 0.7071067)));
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
                SpeedCalculator(XDistance, YDistance, Speed, AccelDistance, DecelDistance, Accel, Decel);
                MecanumFunction(YL, XL, (-MaintainAngle + CurrentHeading) * IMUGain);
                if (isStopRequested()) {
                    break;
                }
            }

            if (Decel) {
                YL = XL = 0;
            }

        } else if (XDistance == 0) {
            while (Math.abs(Yposition) <= Math.abs(YDistance)) {
                if (isStopRequested()) {
                    MecanumFunction(0, 0, 0);
                    break;
                }
                telemetry.addData("YPosition", Yposition);
                telemetry.update();
                // telemetry.addData("Speed", Math.max(Math.min(Speed * ((Math.abs(Yposition) * 0.006) / Accelerate) - 0/*(Decelerate * (1 / (Math.abs(YDistance) - Math.abs(Yposition))))*/, 0.0001), Speed * 0.707106781 * Math.abs((YDistance / XDistance)) * Y));
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                CurrentHeading = angles.firstAngle;
                DistanceTracker();
                SpeedCalculator(XDistance, YDistance, Speed, AccelDistance, DecelDistance, Accel, Decel);
                MecanumFunction(YL, 0, (-MaintainAngle + CurrentHeading) * IMUGain);
            }

            if (Decel) {
                YL = XL = 0;
            }

        }
    }

    private void Neutral() {
        righteye.setPosition(0.5);
        lefteye.setPosition(0.5);
        leftbrow.setPosition(0.5);
        rightbrow.setPosition(0.5);
    }


    private void Right() {
        righteye.setPosition(0.15);
        lefteye.setPosition(0.15);
    }


    private void Mad() {
        leftbrow.setPosition(0.65);
        rightbrow.setPosition(0.3);
    }


    private void Left() {
        righteye.setPosition(0.85);
        lefteye.setPosition(0.85);
    }

    private void Happy() {
        leftbrow.setPosition(0.3);
        rightbrow.setPosition(0.65);
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

    private void ShootPowershots() {
        double currentIMU;
        currentIMU = CurrentHeading;
        GoTov2(9, 0, 0.6, 0, 3, 3, true, true, 0.004, true);
        MecanumFunction(0, 0, 0);
        clarm.setPosition(0.12);
        LeftLauncher.setPower(0.3);
        RightLauncher.setPower(-0.3);
//        sleep(500);
        ramp.setTargetPosition(759);
        ramp.setPower(0.6);
        sleep(500);
        RightLauncher.setPower(0.77);
        sleep(150);
        LeftLauncher.setPower(-0.3775);
        sleep(800);
        AngularAdjustment(currentIMU - 23, 0.012);
        pusher.setPosition(0.41);
        LeftLauncher.setPower(-0.3775);
        RightLauncher.setPower(0.77);
        sleep(520);
        AngularAdjustment(currentIMU - 27, .012);
        pusher.setPosition(0.775);
        sleep(520);
        AngularAdjustment(currentIMU - 31, 0.012);
        pusher.setPosition(0.98);
        sleep(520);
        LeftLauncher.setPower(0);
        RightLauncher.setPower(0);
        pusher.setPosition(0);
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
        CurrentHeading = angles.firstAngle;

//        motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor_drive_blAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor_drive_brAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

            BulkCaching();
            if (((-targetangle + CurrentHeading) >= 0.07 && (-targetangle + CurrentHeading) <= -0.07)) {
                TimerD.reset();
            }
            MecanumFunction(0, 0, IMUgain * (-targetangle + CurrentHeading));
            if (isStopRequested()) {
                break;
            }
        }
        MecanumFunction(0, 0, 0);
    }

    private void Shoot(double RP, double LP, double Timer, boolean Stop) {
        ElapsedTime ShootTimer;
        ShootTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        ShootTimer.reset();
        while (ShootTimer.seconds() <= Timer) {
            LeftLauncher.setPower(LP);
            RightLauncher.setPower(RP);
            Conveyor.setPower(1);
            if (isStopRequested()) {
                break;
            }
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
        //  telemetry.addData("FinalAngle", FinalAngle);
        // telemetry.addData("LastValue", LastValue);
        //telemetry.addData("CurrentValue", angles.firstAngle);
        //telemetry.update();
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
        Yposition = (((((motor_drive_blAsDcMotor.getCurrentPosition() + motor_drive_brAsDcMotor.getCurrentPosition() + motor_drive_flAsDcMotor.getCurrentPosition() + motor_drive_frAsDcMotor.getCurrentPosition()) * 0.25) * 0.00208333333) * 12.8) - YPositionReset);//((((motor_drive_brAsDcMotor.getCurrentPosition() + motor_drive_blAsDcMotor.getCurrentPosition() + motor_drive_flAsDcMotor.getCurrentPosition() + motor_drive_frAsDcMotor.getCurrentPosition()) / 4) / 480) * 12.566);
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
                MecanumFunction(Math.min(Math.max(Speed * ((Math.abs(Yposition) * 0.6) / Accelerate) * (((Math.abs(YDistance) - Math.abs(Yposition)) * 0.4) / Decelerate), 0.05), YSpeed) * Y * YD, Math.min(Math.max(Speed * ((Math.abs(Xposition) * 1) / XAccelerate) * (((Math.abs(XDistance) - Math.abs(Xposition)) * 0.7) / XDecelerate), 0.49), Speed) * X * XD, (-MaintainAngle + CurrentHeading) * IMUGain);


                YSpeed = Math.abs(Speed * 0.707106781 * Math.abs((YDistance / XDistance)) * Y);
                if (isStopRequested()) {
                    break;
                }
            }

            if (Decel) {
                MecanumFunction(0, 0, 0);
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

                MecanumFunction(Math.min(Math.max(Speed * ((Math.abs(Yposition) * 0.6) / Accelerate) * (((Math.abs(YDistance) - Math.abs(Yposition)) * 0.4) / Decelerate), 0.05), Speed) * Y * YD, 0, (-MaintainAngle + CurrentHeading) * IMUGain);
                if (isStopRequested()) {
                    break;
                }
            }

            if (Decel) {
                MecanumFunction(0, 0, 0);
            }


        }
    }

    private void MecanumFunction(double YL, double XL, double XR) {
        if (YL != 0) {
            Range.clip(XL, -0.5, 0.5);
            Range.clip(XR, -0.5, 0.5);
        } else if (XL != 0) {
            Range.clip(YL, -0.5, 0.5);
            Range.clip(XR, -0.5, 0.5);
        } else if (XR != 0) {
            Range.clip(YL, -0.5, 0.5);
            Range.clip(XL, -0.5, 0.5);
        }

        if (YL < 0) {
            YL = -YL * YL;
        } else {
            YL *= YL;
        }

        if (XL < 0) {
            XL = -XL * XL;
        } else {
            XL *= XL;
        }

        if (XR < 0) {
            XR = -XR * XR;
        } else {
            XR *= XR;
        }

        flScale = (-YL - (-XL - XR));
        blScale = (YL - (-XL + XR));
        frScale = (-YL + XL - XR);
        brScale = (YL + XL + XR);


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
            motor_drive_flAsDcMotor.setPower(Range.clip((YL - (-XL - XR)), -1, 1));
            motor_drive_blAsDcMotor.setPower(Range.clip(-(-YL - (-XL + XR)), -1, 1));
            motor_drive_frAsDcMotor.setPower((Range.clip((YL - XL - XR), -1, 1)));
            motor_drive_brAsDcMotor.setPower(Range.clip(-(-YL - XL + XR), -1, 1));
        } else {
            motor_drive_flAsDcMotor.setPower(Range.clip((YL - (-XL - XR)), -1, 1));
            motor_drive_blAsDcMotor.setPower(Range.clip(-(-YL - (-XL + XR)), -1, 1));
            motor_drive_frAsDcMotor.setPower((Range.clip((YL - XL - XR), -1, 1)));
            motor_drive_brAsDcMotor.setPower(Range.clip(-(-YL - XL + XR), -1, 1));

            /*
            motor_drive_flAsDcMotor.setPower((YL - (-XL - XR)));
            motor_drive_blAsDcMotor.setPower(-(-YL - (-XL + XR)));
            motor_drive_frAsDcMotor.setPower((YL - XL - XR));
            motor_drive_brAsDcMotor.setPower(-(-YL - (-XL + XR)));
*/
        }


    }


    private void MagFull(double Yl, double Xl, double Timeout, double MaintainAngle, double IMUGain) {
        if (pusher.getPosition() < 0.5) {
            boolean gamepad2a = false;
            double v = 1;
            boolean gamepad2xafter = false;
            ClawVariable = 1;
            // ANDREW THINK ABOUT WHAT SHOULD BE CHECKED BEFORE MOVING THE RAMP DOWN?
            // ALSO, PROBABLY GOOD TO START SPINNING THE LAUNCHER WHEELS IN AT THE START OF THIS FUNCTION
            if (pusher.getPosition() < 0.5) {
                ramp.setTargetPosition(0);
                ramp.setPower(-0.5);
            }

            /*while (ramp.isBusy()) {
                if (gamepad1.left_trigger > 0.3){
                    MecanumFunction(-gamepad1.left_stick_y * 0.5, gamepad1.left_stick_x * 0.5, gamepad1.right_stick_x * 0.5);
                } else {
                    MecanumFunction(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                }
                if (isStopRequested()) {
                    break;
                }
            }*/
            ElapsedTime MagTimer = new ElapsedTime();
            while (MagTimer.seconds() < Timeout) {
                if (gamepad2.left_bumper) {
                    intake.setPower(-0.6);
                } else {
                    intake.setPower(0.6);
                }
                if (gamepad2.dpad_up) {
                    break;
                }
                /*if (gamepad2.dpad_right) {
                    IntakeCurrentState = State.BREAK;
                }
                if (gamepad2.dpad_up){
                    IntakeCurrentState = State.MOVE_TO_SHOOT;
                    break;
                }*/
                gamepad2a = gamepad2.a;
                if (gamepad2.a && !gamepad2a) {
                    v *= -1;
                }
                if (v > 0) {
                    if (gamepad2.x && !gamepad2xafter) {
                        clarmvariable = clarmvariable * -1;
                    }
                    if (clarmvariable > 0 && clarmvariable < 5) {
                        clarm.setPosition(0.08);
                    } else if (clarmvariable > -5 && clarmvariable < 0) {
                        clarm.setPosition(0.55);
                    }
                } else {
                    clarm.setPosition(0);
                }

                if (gamepad2.b && !gamepadbAfter) {
                    ClawVariable = ClawVariable * -1;
                }
                if (ClawVariable > 0) {
                    claw.setPosition(0);
                } else if (ClawVariable < 0) {
                    claw.setPosition(1);
                }
                gamepadbAfter = gamepad2.b;

                gamepad2xafter = gamepad2.x;
                if (gamepad1.left_trigger > 0.3) {
                    MecanumFunction(-gamepad1.left_stick_y * 0.5 / 2, gamepad1.left_stick_x * 0.5 / 2, gamepad1.right_stick_x * 0.5 / 2);
                } else {
                    MecanumFunction(-gamepad1.left_stick_y / 2, gamepad1.left_stick_x / 2, gamepad1.right_stick_x / 2);
                }

                normalizedColors = ((NormalizedColorSensor) rangeSensor_REV_ColorRangeSensor).getNormalizedColors();
                telemetry.addData("Hue", Double.parseDouble(JavaUtil.formatNumber(normalizedColors.red, 3)));
                telemetry.update();
                LeftLauncher.setPower(0.15);
                RightLauncher.setPower(-0.15);
                if (Double.parseDouble(JavaUtil.formatNumber(normalizedColors.red, 3)) < 0.0175) {
                    MagTimer.reset();
                }
            }
            FullMag = true;
            //MecanumFunction(0, 0, 0, 0);
            intake.setPower(-.25);

        /*ramp.setTargetPosition(822);
        ramp.setPower(0.7);
        sleep(1000);*/
        }
    }

    private void IntakeAndShoot() {
        switch (IntakeAndShootCurrentState) {
            case INTAKE_IDLE:
                if ((!gamepad2.y) && (!gamepad2.dpad_right) && (!gamepad2.dpad_up) && (!gamepad2.dpad_left) && (gamepad2.left_trigger < 0.3) && (VPTimer == true)) {
                    RightLauncher.setPower(0);
                    LeftLauncher.setPower(0);
                    pusher.setPosition(0);
                    ramp.setTargetPosition(0);
                    ramp.setPower(-0.2);
                    intake.setPower(0);
                    telemetry.addData("idle", IntakeCurrentState);
                    telemetry.update();
                } else if (gamepad2.y) {
                    IntakeAndShootCurrentState = State.DISC_COLLECTION;
                } else if (gamepad2.dpad_up || FullMag == true) {
                    IntakeAndShootCurrentState = State.RAMP_TO_SHOOT;
                } else if (gamepad2.dpad_left) {
                    IntakeAndShootCurrentState = State.POWERSHOT_RAMP;
                } else {    //ANDREW, in other code below, there is a check before going to SHOOT_2, why is this not done here?
                    IntakeAndShootCurrentState = State.SHOOT_2;
                }
                break;

            case DISC_COLLECTION:
                if (gamepad2.y) {
                    LeftLauncher.setPower(0.1);
                    RightLauncher.setPower(-0.1);
                    MagFull(0, 0, 0.35, FinalAngle, 0.006);
                    telemetry.addData("intake", IntakeCurrentState);
                    telemetry.update();
                    if (gamepad2.dpad_up) {
                        IntakeAndShootCurrentState = State.RAMP_TO_SHOOT;
                    }
                } else if ((((Math.abs(840 - ramp.getCurrentPosition())) < 5) || (Math.abs(835 - ramp.getCurrentPosition())) < 5) && !VPTimer) {
                    IntakeAndShootCurrentState = State.SHOOT_2;
                } else if (gamepad2.dpad_up || FullMag == true) {
                    IntakeAndShootCurrentState = State.RAMP_TO_SHOOT;
                } else if (gamepad2.dpad_left) {
                    IntakeAndShootCurrentState = State.POWERSHOT_RAMP;
                } else {
                    IntakeAndShootCurrentState = State.INTAKE_IDLE;
                }
                break;

            case RAMP_TO_SHOOT:
                if (gamepad2.dpad_up || FullMag == true) {
                    MecanumFunction(-gamepad1.left_stick_y / 2, gamepad1.left_stick_x / 2, gamepad1.right_stick_x / 2);
                    ramp.setTargetPosition(840);
                    ramp.setPower(0.7);
                    LeftLauncher.setPower(0.1);
                    RightLauncher.setPower(-0.1);
                    while (ramp.isBusy()) {   // ANDREW, can you use ramp.isBusy() instead of this compare?  It would be a better way to handle this
                        MecanumFunction(-gamepad1.left_stick_y / 2, gamepad1.left_stick_x / 2, gamepad1.right_stick_x / 2);
                    }
                    FullMag = false;
                    telemetry.addData("move to shoot", IntakeCurrentState);
                    telemetry.update();
                    VPTimer = false;
                    //PusherTimer.reset();
                } else if ((((Math.abs(840 - ramp.getCurrentPosition())) < 5)) && !VPTimer) {
                    IntakeAndShootCurrentState = State.SHOOT_2;
                } else if (gamepad2.dpad_left) {
                    IntakeAndShootCurrentState = State.RAMP_TO_SHOOT;
                } else if (gamepad2.y) {
                    IntakeAndShootCurrentState = State.DISC_COLLECTION;
                } else {
                    IntakeAndShootCurrentState = State.INTAKE_IDLE;
                }
                break;

            case POWERSHOT_RAMP:
                if (gamepad2.dpad_left) {
                    MecanumFunction(-gamepad1.left_stick_y / 2, gamepad1.left_stick_x / 2, gamepad1.right_stick_x / 2);
                    ramp.setTargetPosition(835);
                    ramp.setPower(0.5);
                    LeftLauncher.setPower(0.1);
                    RightLauncher.setPower(-0.1);
                    while (ramp.getCurrentPosition() < 800) {
                        MecanumFunction(-gamepad1.left_stick_y / 2, gamepad1.left_stick_x / 2, gamepad1.right_stick_x / 2);
                        FullMag = true;
                    }
                    FullMag = false;
                    telemetry.addData("powershot", IntakeCurrentState);
                    telemetry.update();
                } else if ((((Math.abs(840 - ramp.getCurrentPosition())) < 5) || (Math.abs(835 - ramp.getCurrentPosition())) < 5) && !VPTimer) {
                    IntakeAndShootCurrentState = State.SHOOT_2;
                } else if (gamepad2.dpad_up || FullMag == true) {
                    IntakeAndShootCurrentState = State.RAMP_TO_SHOOT;
                } else if (gamepad2.y) {
                    IntakeAndShootCurrentState = State.DISC_COLLECTION;
                } else {
                    IntakeAndShootCurrentState = State.INTAKE_IDLE;
                }
                break;

            case SHOOT_2:
                if (((Math.abs(840 - ramp.getCurrentPosition())) < 5) && !VPTimer) {
                    intake.setPower(0);
                    LeftLauncher.setPower(-0.4);
                    RightLauncher.setPower(0.8);
                    //      ElapsedTime PusherTimer = new ElapsedTime();
                    while (!VPTimer) {   //ANDREW, you should probably not start this timer until the left_trigger is pressed.  Also, you are not able to move when you are in this state...  This is a problem
                        telemetry.addData("pusher position", PusherTimer.milliseconds());
                        telemetry.update();
                        MecanumFunction(-gamepad1.left_stick_y / 2, gamepad1.left_stick_x / 2, gamepad1.right_stick_x / 2);
                        if (gamepad2.left_trigger > 0.3 && ramp.getCurrentPosition() > 300) {
                            pusher.setPosition(0.98);
                            ElapsedTime PusherTimer = new ElapsedTime();

                        }
                        if (!((((Math.abs(840 - ramp.getCurrentPosition())) < 5) || (Math.abs(835 - ramp.getCurrentPosition())) < 5) && !VPTimer)) {
                            break;
                        }
                        if (pusher.getPosition() < 0.5) {
                            PusherTimer.reset();
                        }
                        if (PusherTimer.milliseconds() > 1800) {
                            VPTimer = true;
                        }
                        telemetry.addData("pusher position", PusherTimer.milliseconds());
                        telemetry.update();
                        telemetry.addData("shoot", IntakeCurrentState);
                        telemetry.update();
                    }

                } else if (gamepad2.dpad_left) {
                    IntakeAndShootCurrentState = State.POWERSHOT_RAMP;
                } else if (gamepad2.dpad_up || FullMag == true) {
                    IntakeAndShootCurrentState = State.RAMP_TO_SHOOT;
                } else if (gamepad2.y) {
                    IntakeAndShootCurrentState = State.DISC_COLLECTION;
                } else {
                    IntakeAndShootCurrentState = State.INTAKE_IDLE;
                }
                break;
        }
    }

    /*private void IntakeStateMachine() {
        boolean gamepad1Yafter = false;
        boolean gamepad2dpadleftafter = false;
        boolean gamepad2dpaddownafter = false;
        switch (IntakeCurrentState) {
            case DISC_COLLECTION:
                if (gamepad2.y) {
                    LeftLauncher.setPower(0.1);
                    RightLauncher.setPower(-0.1);
                    MagFull(0,0,0.35, FinalAngle,0.006);
                    if (gamepad2.dpad_left && !gamepad2dpadleftafter){
                        n  = 1;
                    }
                    gamepad2dpadleftafter = gamepad2.dpad_left;
                    if (gamepad2.dpad_down && !gamepad2dpaddownafter){
                        n = -1;
                    }
                    gamepad2dpaddownafter = gamepad2.dpad_down;
                    if (n == 1){
                        ramp.setTargetPosition(840);
                        ramp.setPower(0.8);
                    }
                    if (n == -1){
                        ramp.setTargetPosition(800);
                        ramp.setPower(0.8);
                    }
                    while(ramp.isBusy()){

                        if (gamepad2.right_trigger > 0.3){
                            pusher.setPosition(0);
                        }
                        if (gamepad2.left_trigger > 0.3){
                            pusher.setPosition(0.98);
                        }
                        if (gamepad1.x){
                            pusher.setPosition(0.41);
                        }
                        if(gamepad1.y){
                            pusher.setPosition(0.775);
                        }
                        if (gamepad1.b){
                            pusher.setPosition(0.98);
                        }
                        gamepad1Yafter = gamepad1.y;
                        LeftLauncher.setPower(0.1);
                        RightLauncher.setPower(-0.1);
                        if (gamepad1.left_trigger > 0.3){
                           // MecanumFunction(-gamepad1.left_stick_y * 0.5, gamepad1.left_stick_x * 0.5, gamepad1.right_stick_x * 0.5);
                        } else {
                           // MecanumFunction(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                        }
                        if (isStopRequested()) {
                            break;
                        }
                        if (gamepad2.dpad_left && !gamepad2dpadleftafter){
                            n = 1;
                        }
                        gamepad2dpadleftafter = gamepad2.dpad_left;
                        if (gamepad2.dpad_down && !gamepad2dpaddownafter){
                            n = -1;
                        }
                        gamepad2dpaddownafter = gamepad2.dpad_down;
                        if (n == 1){
                            ramp.setTargetPosition(840);
                            ramp.setPower(0.8);
                        }
                        if (n == -1){
                            ramp.setTargetPosition(800);
                            ramp.setPower(0.8);
                        }
                    }
                    LeftLauncher.setPower(0);
                    RightLauncher.setPower(0);
                    telemetry.addData("Disc collection", IntakeCurrentState);
                    telemetry.update();

                    // ANDREW CHECK IF THIS && IS THE CORRECT OPERATOR
                } else if ((gamepad2.dpad_up || FullMag == true) && !gamepad2.dpad_right && !gamepad2.y) {
                    IntakeCurrentState = State.MOVE_TO_SHOOT;
                } else if (gamepad2.dpad_right) {
                    IntakeCurrentState = State.BREAK;
                } else if (gamepad2.dpad_left) {
                    IntakeCurrentState = State.SHOOT_POWERSHOT;
                } else {
                    IntakeCurrentState = State.INTAKE_IDLE;
                }
                break;

            case MOVE_TO_SHOOT:
                // ANDREW CHECK IF THIS && IS THE CORRECT OPERATOR
                ramp.setPower(0.1);
                if ((gamepad2.dpad_up || FullMag == true) && !gamepad2.dpad_right && !gamepad2.y) {
                    clarm.setPosition(0.12);
                    while (ramp.isBusy()) {
                        if (gamepad2.right_trigger > 0.3) {
                            pusher.setPosition(0);
                        }
                        if (gamepad2.left_trigger > 0.3) {
                            pusher.setPosition(0.98);
                        }
                        if (gamepad1.x) {
                            pusher.setPosition(0.41);
                        }
                        if (gamepad1.y) {
                            pusher.setPosition(0.775);
                        }
                        if (gamepad1.b) {
                            pusher.setPosition(0.98);
                        }
                        if (gamepad2.dpad_left && !gamepad2dpadleftafter) {
                            n = 1;
                        }
                        gamepad2dpadleftafter = gamepad2.dpad_left;
                        if (gamepad2.dpad_down && !gamepad2dpaddownafter) {
                            n = -1;
                        }
                        gamepad2dpaddownafter = gamepad2.dpad_down;
                        if (n == 1) {
                            if (n == 1) {
                                ramp.setTargetPosition(840);
                                ramp.setPower(0.8);
                            }
                            if (n == -1) {
                                ramp.setTargetPosition(800);
                                ramp.setPower(0.8);
                            }
                            gamepad1Yafter = gamepad1.y;
                            LeftLauncher.setPower(0.1);
                            RightLauncher.setPower(-0.1);
                            if (gamepad1.left_trigger > 0.3) {
                             //   MecanumFunction(-gamepad1.left_stick_y * 0.5, gamepad1.left_stick_x * 0.5, gamepad1.right_stick_x * 0.5);
                            } else {
                           //     MecanumFunction(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                            }
                            if (isStopRequested()) {
                                break;
                            }
                        }
                        if (gamepad2.dpad_left && !gamepad2dpadleftafter) {
                            n = 1;
                        }
                        gamepad2dpadleftafter = gamepad2.dpad_left;
                        if (gamepad2.dpad_down && !gamepad2dpaddownafter) {
                            n = -1;
                        }
                        gamepad2dpaddownafter = gamepad2.dpad_down;
                        if (n == 1) {
                            ramp.setTargetPosition(840);
                            ramp.setPower(0.8);
                        }
                        if (n == -1) {
                            ramp.setTargetPosition(800);
                            ramp.setPower(0.8);
                        }
                        //sleep(500);
                        telemetry.addData("Move to shoot", IntakeCurrentState);
                        telemetry.update();
                    }
                } else if (gamepad2.y){
                    IntakeCurrentState = State.DISC_COLLECTION;
                } else if (gamepad2.dpad_right) {
                    IntakeCurrentState = State.BREAK;
                } else if (gamepad2.dpad_left) {
                    IntakeCurrentState = State.SHOOT_POWERSHOT;
                }else {
                    IntakeCurrentState = State.INTAKE_IDLE;
                }
                break;

            case SHOOT_POWERSHOT:
                /*
                if (gamepad2.dpad_left) {

                } else if (gamepad2.y){
                    IntakeCurrentState = State.DISC_COLLECTION;
                } else if (gamepad2.dpad_right) {
                    IntakeCurrentState = State.BREAK;
                    // ANDREW THIS ELSE IF CONDITION DOESN'T LOOK RIGHT.  SHOULD BE ABLE TO MOVE TO SHOOT POWERSHOT INDEPENDENT OF THE FULLMAG CONDITION
                } else if ((gamepad2.dpad_up || FullMag == true) && !gamepad2.dpad_right && !gamepad2.y) {
                    IntakeCurrentState = State.MOVE_TO_SHOOT;
                }else {
                    IntakeCurrentState = State.INTAKE_IDLE;
                }


                break;

            case BREAK:
                if (gamepad2.dpad_right) {
                    LeftLauncher.setPower(0);
                    RightLauncher.setPower(0);

                    // ANDREW ETHAN SAID THAT HE WOULD LIKE THE RAMP TO NOT MOVE DURING BREAK ACTUALLY, BREAK STATE IS BASICALLY THE SAME AS INTAKE IDLE SO MIGHT BE A SUPERFLUOUS STATE
                    telemetry.addData("Break", IntakeCurrentState);
                    telemetry.update();
                }
                else if (gamepad2.y) {
                    IntakeCurrentState = State.DISC_COLLECTION;
                } else if ((FullMag = true || gamepad2.dpad_up) && !gamepad2.dpad_right && !gamepad2.y) {
                    IntakeCurrentState = State.MOVE_TO_SHOOT;
                } else if (gamepad2.dpad_left) {
                    IntakeCurrentState = State.SHOOT_POWERSHOT;
                }else {
                    IntakeCurrentState = State.INTAKE_IDLE;
                }
                break;

            case INTAKE_IDLE:
                if ((!FullMag) && (!gamepad2.y) && (!gamepad2.dpad_right) && (!gamepad2.dpad_up) && (!gamepad2.dpad_left) && (ramp.getCurrentPosition() < 100)) {
                    //if (pusher.getPosition() != 0) {
                   //     pusher.setPosition(0);
                 //   }
                    LeftLauncher.setPower(0);
                    RightLauncher.setPower(0);

                // ANDREW PROBABLY WANT TO SHUT OFF A COUPLE MORE THINGS DURING THIS IDLE STATE, WHAT ELSE SHOULD BE STOPPED?
                    intake.setPower(0);
                    telemetry.addData("Idle", IntakeCurrentState);
                    telemetry.update();
                } else if (gamepad2.y) {
                    IntakeCurrentState = State.DISC_COLLECTION;
                } else if (gamepad2.dpad_right) {
                    IntakeCurrentState = State.BREAK;
                } else if (gamepad2.dpad_left) {
                    IntakeCurrentState = State.SHOOT_POWERSHOT;
                } else if ((FullMag == true || gamepad2.dpad_up) && !gamepad2.dpad_right && !gamepad2.y){
                    IntakeCurrentState = State.MOVE_TO_SHOOT;
                }
                break;
        }

    }*/

    private void PersonalityStateMachine() {
        switch (PersonalityCurrentState) {
            case MOVE_RIGHT:
                if (gamepad1.right_stick_x > 0 || gamepad1.left_stick_x > 0) {
                    Right();
                    leftbrow.setPosition(0.5);
                    rightbrow.setPosition(0.5);
                } else if (gamepad1.right_stick_x < 0 || gamepad1.left_stick_x < 0) {
                    PersonalityCurrentState = State.MOVE_LEFT;
                } else if (gamepad1.left_stick_y < 0) {
                    PersonalityCurrentState = State.MOVE_FORWARD;
                } else if (gamepad1.left_stick_y > 0) {
                    PersonalityCurrentState = State.MOVE_BACKWARDS;
                } else if (RightLauncher.getPower() > 0) {
                    PersonalityCurrentState = State.SHOOT;
                } else if (intake.getPower() > 0) {
                    PersonalityCurrentState = State.INTAKE;
                } else {
                    PersonalityCurrentState = State.PERSONALITY_IDLE;
                }
                break;


            case MOVE_LEFT:
                if (gamepad1.right_stick_x < 0 || gamepad1.left_stick_x < 0) {
                    Left();
                    leftbrow.setPosition(0.5);
                    rightbrow.setPosition(0.5);
                } else if (gamepad1.right_stick_x > 0 || gamepad1.left_stick_x > 0) {
                    PersonalityCurrentState = State.MOVE_RIGHT;
                } else if (gamepad1.left_stick_y < 0) {
                    PersonalityCurrentState = State.MOVE_FORWARD;
                } else if (gamepad1.left_stick_y > 0) {
                    PersonalityCurrentState = State.MOVE_BACKWARDS;
                } else if (RightLauncher.getPower() > 0) {
                    PersonalityCurrentState = State.SHOOT;
                } else if (intake.getPower() > 0) {
                    PersonalityCurrentState = State.INTAKE;
                } else {
                    PersonalityCurrentState = State.PERSONALITY_IDLE;
                }
                break;

            case MOVE_FORWARD:
                if (gamepad1.left_stick_y < 0) {
                    Neutral();
                } else if (gamepad1.right_stick_x > 0 || gamepad1.left_stick_x > 0) {
                    PersonalityCurrentState = State.MOVE_RIGHT;
                } else if (gamepad1.right_stick_x < 0 || gamepad1.left_stick_x < 0) {
                    PersonalityCurrentState = State.MOVE_LEFT;
                } else if (gamepad1.left_stick_y > 0) {
                    PersonalityCurrentState = State.MOVE_BACKWARDS;
                } else if (RightLauncher.getPower() > 0) {
                    PersonalityCurrentState = State.SHOOT;
                } else if (intake.getPower() > 0) {
                    PersonalityCurrentState = State.INTAKE;
                } else {
                    PersonalityCurrentState = State.PERSONALITY_IDLE;
                }
                break;

            case MOVE_BACKWARDS:
                if (gamepad1.left_stick_y > 0) {
                    Left();
                } else if (gamepad1.right_stick_x > 0 || gamepad1.left_stick_x > 0) {
                    PersonalityCurrentState = State.MOVE_RIGHT;
                } else if (gamepad1.right_stick_x < 0 || gamepad1.left_stick_x < 0) {
                    PersonalityCurrentState = State.MOVE_LEFT;
                } else if (gamepad1.left_stick_y < 0) {
                    PersonalityCurrentState = State.MOVE_FORWARD;
                } else if (RightLauncher.getPower() > 0) {
                    PersonalityCurrentState = State.SHOOT;
                } else if (intake.getPower() > 0) {
                    PersonalityCurrentState = State.INTAKE;
                } else {
                    PersonalityCurrentState = State.PERSONALITY_IDLE;
                }
                break;

            case SHOOT:
                if (RightLauncher.getPower() > 0) {
                    Mad();
                    lefteye.setPosition(0.5);
                    righteye.setPosition(0.5);
                } else if (gamepad1.right_stick_x > 0 || gamepad1.left_stick_x > 0) {
                    PersonalityCurrentState = State.MOVE_RIGHT;
                } else if (gamepad1.right_stick_x < 0 || gamepad1.left_stick_x < 0) {
                    PersonalityCurrentState = State.MOVE_LEFT;
                } else if (gamepad1.left_stick_y < 0) {
                    PersonalityCurrentState = State.MOVE_FORWARD;
                } else if (gamepad1.left_stick_y > 0) {
                    PersonalityCurrentState = State.MOVE_BACKWARDS;
                } else if (intake.getPower() > 0) {
                    PersonalityCurrentState = State.INTAKE;
                } else {
                    PersonalityCurrentState = State.PERSONALITY_IDLE;
                }
                break;

            case INTAKE:
                if (gamepad1.right_stick_x > 0 || gamepad1.left_stick_x > 0) {
                    PersonalityCurrentState = State.MOVE_RIGHT;
                } else if (gamepad1.right_stick_x < 0 || gamepad1.left_stick_x < 0) {
                    PersonalityCurrentState = State.MOVE_LEFT;
                } else if (gamepad1.left_stick_y < 0) {
                    PersonalityCurrentState = State.MOVE_FORWARD;
                } else if (gamepad1.left_stick_y > 0) {
                    PersonalityCurrentState = State.MOVE_BACKWARDS;
                } else if (intake.getPower() > 0) {
                    PersonalityCurrentState = State.INTAKE;
                } else {
                    PersonalityCurrentState = State.PERSONALITY_IDLE;
                }
                break;
               /* //if (intake.getPower() > 0) {

                    telemetry.addData("intake", CurrentState);
                    telemetry.update();
                 //   Happy();
                } else if (gamepad1.right_stick_x > 0 || gamepad1.left_stick_x > 0) {
                    CurrentState = State.MOVE_RIGHT;
                } else if (gamepad1.right_stick_x < 0 || gamepad1.left_stick_x < 0) {
                    CurrentState = State.MOVE_LEFT;
                } else if (gamepad1.left_stick_y < 0) {
                    CurrentState = State.MOVE_FORWARD;
                } else if (gamepad1.left_stick_y > 0) {
                    CurrentState = State.MOVE_BACKWARDS;
                } else if (RightLauncher.getPower() > 0) {
                    CurrentState = State.SHOOT;
                } else if (gamepad2.y) {
                    CurrentState = State.MAD;
                } else {
                    CurrentState = State.IDLE;
                }
                break;
*/
            case PERSONALITY_IDLE:
                if ((RightLauncher.getPower() == 0) && (intake.getPower() == 0) && (gamepad1.right_stick_x == 0) && (gamepad1.left_stick_y == 0) && (gamepad1.left_stick_x == 0)) {
                    Happy();
                } else if (gamepad1.right_stick_x > 0 && gamepad1.left_stick_x > 0) {
                    PersonalityCurrentState = State.MOVE_RIGHT;
                } else if (gamepad1.right_stick_x < 0 && gamepad1.left_stick_x < 0) {
                    PersonalityCurrentState = State.MOVE_LEFT;
                } else if (gamepad1.left_stick_y < 0) {
                    PersonalityCurrentState = State.MOVE_FORWARD;
                } else if (gamepad1.left_stick_y > 0) {
                    PersonalityCurrentState = State.MOVE_BACKWARDS;
                } else if (RightLauncher.getPower() > 0) {
                    PersonalityCurrentState = State.SHOOT;
                } else if (gamepad2.y) {
                    PersonalityCurrentState = State.MAD;
                } else {
                    PersonalityCurrentState = State.INTAKE;
                }
                break;

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




