package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/* TODO
- Check to make sure HW reads are being done efficiently.  Don't ask for hardware reads more often than required
- Consider adding a smoothing feature in MecanumFunction
- Fix the motor names.  THey are very awkward right now
- Confirm that WallTracking exits properly
- Add camera direction function into WallTracking
- Tune motors
    -Backright motor not tuned correctly. Wall tracking program bugging out. Not getting to desired location, but does exit the loop when robot is manually guided to Target Position.
- Create exact angular adjustment function
 */

@Autonomous(name = "BestSTATEAutoEver", group = "")

public class BestSTATEAutoEver extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private DcMotor RightLauncher;
    private DcMotor LeftLauncher;
    private CRServo Conveyor;
    private DcMotor intake;
    public static final double NEW_P = 20;
    public static final double NEW_I = 15;
    public static final double NEW_D = 0;
    public static final double NEW_F = 5;
    private DcMotor clarm;
    private Servo claw;
    private Servo ramp;
    private static final String VUFORIA_KEY =
            "Ae2mEyz/////AAABmQBmoTE94ki5quwzTT/OlIIeOueUfjuHL/5k1VNWN943meU2RmiXCJ9eX3rUR/2CkwguvbBU45e1SzrbTAwz3ZzJXc7XN1ObKk/7yPHQeulWpyJgpeZx+EqmZW6VE6yG4mNI1mshKI7vOgOtYxqdR8Yf7YwBPd4Ruy3NVK01BwBl1F8V/ndY26skaSlnWqpibCR3XIvVG0LXHTdNn/ftZyAFmCedLgLi1UtNhr2eXZdr6ioikyRYEe7qsWZPlnwVn5DaQoTcgccZV4bR1/PEvDLn7jn1YNwSimTC8glK+5gnNpO+X7BiZa5LcqtYEpvk/QNQda0Fd+wHQDXA8ojeMUagawtkQGJvpPpz9c6p4fad";
    private static final float mmPerInch = 25.4f;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private static final float mmTargetHeight = (6) * mmPerInch;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;
    private OpenGLMatrix lastLocation = null;
    //WebcamName webcamName = null;
    private DcMotorEx motor_drive_flAsDcMotor, motor_drive_blAsDcMotor, motor_drive_brAsDcMotor, motor_drive_frAsDcMotor;
    private BNO055IMU imu;
    private VuforiaCurrentGame vuforiaUltimateGoal;
    Orientation angles;
    ElapsedTime TimerA;
    ElapsedTime TimerC;
    double OneVote, NoneVote, QuadVote;
    double DecelConstant, DecelConstant2;
    ElapsedTime TimerD;
    ElapsedTime MagTimer;
    ElapsedTime AccelTimer;
    ElapsedTime DevelTimer;
    float CurrentHeading;
    ElapsedTime TimerB;
    double CurrentVal;
    double LastVal;
    double QuadRun;
    double OneRun;
    double YL, XL;
    double NoneRun;
    double YEncoderTicks;
    double AvgReadingY, AvgReadingX;
    double XEncoderTicks;
    double EncoderTicks;
    double YSpeed;
    double YPositionReset, XPositionReset, XPositionP, YPositionP;
    double XDecelerate;
    double TotalEncoderTicks;
    double Accelerate, Decelerate, XAccelerate;
    double mFr, mFl, mBl, mBr;
    double X, Y, XD, YD;
    double CurrentX, CurrentY;
    double mXR, mYL, mXL, flScale, frScale, blScale, brScale, YLin, XLin, XRin;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 90;
    double Heading;
    double Zrot = 0.0;
    double TrueTrackSwitch;
    double IMUTrackSwitch;
    double avgEnc;
    double Xposition;
    double Yposition;
    double AccelConstant, AccelConstant2;
    boolean frOverload, flOverload, blOverload, brOverload;
    boolean fullMag;
    private Servo camServo;
    ModernRoboticsI2cRangeSensor rangeSensor;
    private State CurrentState;

    private enum State {
        MOVE_RIGHT,
        MOVE_LEFT,
        MOVE_FORWARD,
        MOVE_BACKWARDS,
        SHOOT,
        INTAKE,
        IDLE,
    }

    private class PersonalityStateMachine implements Runnable {
        public PersonalityStateMachine() {

        }

        public void run() {
            while (opModeIsActive()) {
                PersonalityStateMachine();
            }
        }
    }

    @Override
    public void runOpMode() {

        // vuforiaUltimateGoal = new VuforiaCurrentGame();
        motor_drive_flAsDcMotor = hardwareMap.get(DcMotorEx.class, "motor_drive_flAsDcMotor");
        motor_drive_frAsDcMotor = hardwareMap.get(DcMotorEx.class, "motor_drive_frAsDcMotor");
        motor_drive_blAsDcMotor = hardwareMap.get(DcMotorEx.class, "motor_drive_blAsDcMotor");
        motor_drive_brAsDcMotor = hardwareMap.get(DcMotorEx.class, "motor_drive_brAsDcMotor");
        RightLauncher = hardwareMap.get(DcMotor.class, "RightLauncher");
        LeftLauncher = hardwareMap.get(DcMotor.class, "LeftLauncher");
        Conveyor = hardwareMap.get(CRServo.class, "Conveyor");
        intake = hardwareMap.get(DcMotor.class, "intake");
        clarm = hardwareMap.get(DcMotor.class, "clarm");
        claw = hardwareMap.get(Servo.class, "claw");
        ramp = hardwareMap.get(Servo.class, "ramp");

        PIDFCoefficients pidOrig = motor_drive_flAsDcMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        Initialization();

        // Activate here for camera preview.
        telemetry.addData(">>", "Vuforia initialized, press start to continue...");
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
        final float CAMERA_FORWARD_DISPLACEMENT = -2.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 5.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 9.0f;     // eg: Camera is ON the robot's center line
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, 90, 0, 90));


        /**  Let all the trackable listeners know where the phone is.  */

        targetVisible = false;

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(3, 1.78);
        }
        claw.setPosition(0);
        targetsUltimateGoal.activate();
        X = 1;
        Y = 1;
        YPositionReset = 0;
        XPositionReset = 0;
        motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_blAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_brAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_blAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_brAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ringScan();
        CurrentState = State.IDLE;
        PersonalityStateMachine myThread = new PersonalityStateMachine();
        myThread.run();
        waitForStart();
        GoTov2(0, 25, 0.3, 0, 9, 9, true, true, 0.012, false);
        //Shooting Powershots Here!
        if (OneRun == 1) {
            GoTov2(0, 20, 0.25, 0, 9, 9, true, true, 0.012, false);
        }
        if (NoneRun == 1) {
            clarm.setTargetPosition(800);
            clarm.setPower(0.4);
            IMUTurn(38);
            GoTov2(0, 33, 0.4, 45, 5, 5, true, true, 0.004, false);
            IMUTurn(130);
            clarm.setTargetPosition(900);
            clarm.setPower(0.3);
            claw.setPosition(1);
            clarm.setTargetPosition(600);
            clarm.setPower(-0.6);
            IMUTurn(0);
            GoTov2(0, -13.25, 0.4, 0, 5, 5, true, true, 0.012, true);
            IMUTurn(-55);
            clarm.setTargetPosition(960);
            clarm.setPower(0.3);
            sleep(500);
            claw.setPosition(0);
            sleep(500);
            clarm.setTargetPosition(450);
            clarm.setPower(-0.6);
            sleep(500);
        }



/*
        GoTov2(0,24, 0.25, 0, 6,6,true,false,0.012,true);
        clarm.setTargetPosition(500);
        clarm.setPower(0.5);
        GoTov2(0,24, 0.25, 0, 6,6,false,false,0.012,true);
        clarm.setTargetPosition(0);
        clarm.setPower(-0.5);
        GoTov2(0,24, 0.25, 0, 6,6,false,true,0.012,true);
        GoTov2(0,-24, 0.25, 0, 6,6,true,false,0.012,true);
        clarm.setTargetPosition(500);
        clarm.setPower(0.5);
        GoTov2(0,-24, 0.25, 0, 6,6,false,false,0.012,true);
        clarm.setTargetPosition(0);
        clarm.setPower(-0.5);
        GoTov2(0,-24, 0.25, 0, 6,6,false,true,0.012,true);
        sleep(1000);
*/


  /*wallTargetTracking(vuforia, allTrackables, 90, 0, 58, 0, 10, 2, 1, 3, false, 0);
 /*
        AngularAdjustment(-6.1, 0.02);

        AngularAdjustment(-3.5, 0.028);

        AngularAdjustment(1.15, 0.028);

        GoTo(0, 12,.2, 0, 14, 0, true, true, 0.012, true);
        if (QuadRun == 1) {
            intakeFunction(16, 1300, 0, true, .03);
            intakeFunction(6, 1300, 0, true, .03);
        }
/*
        GoTo(0, 10,.2, 0, 14, 0, true, false, 0.012, true);
        clarm.setTargetPosition(650);
        clarm.setPower(0.3);
        GoTo(10,10,0.2,0,14,0, false,true,0.012,true);
        GoTo(-10,-10,0.2,0,14,0, true,false,0.012,true);
        clarm.setTargetPosition(0);
        clarm.setPower(-0.3);
        GoTo(0, -10,.2, 0, 14, 0, false, true, 0.012, true);
   /* GoTo(20, 20,.2, 0, 14, 0, true, true, 0.012, true);
        GoTo(-20, -20,.2, 0, 14, 0, true, true, 0.012, true);
        GoTo(-20, 20,.2, 0, 14, 0, true, true, 0.012, true);
        GoTo(20, -20,.2, 0, 14, 0, true, true, 0.012, true);
        GoTo(-20, -20,.2, 0, 14, 0, true, true, 0.012, true);
        GoTo(20, 20,.2, 0, 14, 0, true, true, 0.012, true);
        GoTo(20, -20,.2, 0, 14, 0, true, true, 0.012, true);
        GoTo(-20, 20,.2, 0, 14, 0, true, true, 0.012, true);
*/




    /*
    GoTo(0, 40, .3, 0, 14, 0, false, true, 0.012, true);
IMUTurn(175);
*/

    /*
    GoTo(20, -20, .3, 0, 14, 0, true, true, 0.012, true);
        GoTo(20, 20, .3, 0, 14, 0, true, true, 0.012,true);
        GoTo(0, -20, .3, 0, 14, 0, true, true, 0.012, true);
        GoTo(-20, 20, .3, 0, 14, 0, true, true, 0.012, true);
        GoTo(-20, -20, .3, 0, 14, 0, true, true, 0.012, true);
        GoTo(0, 20, .3, 0, 14, 0, true, true, 0.012,true);

*/

        /*while(Xposition <= 24 || Yposition <= 48){
    if (Xposition >= 24){
        X = 0;
    }
    if (Yposition >= 48){
        Y = 0;
    }
    MecanumFunction(0.1414 * Y,0.1 * X,0);
    DistanceTracker();

}

         */
        MecanumFunction(0, 0, 0);
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

    private void WallTargetGoTo(VuforiaLocalizer vufor, Iterable<? extends VuforiaTrackable> allTrackables, double xTarget, double yTarget, double Speed) {
        targetVisible = false;
        while (targetVisible == false) {
            for (int count = 0; count < 10; count++) {
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
                if (targetVisible) {
                    VectorF translation = lastLocation.getTranslation();
                    CurrentY = translation.get(1) / mmPerInch;
                    CurrentX = translation.get(0) / mmPerInch;
                    AvgReadingY += CurrentY;
                    AvgReadingX += CurrentX;
                }


            }
            AvgReadingY = (AvgReadingY / 10);
            AvgReadingX = (AvgReadingX / 10);
            // Provide feedback as to where the robot is located (if we know).

        }
//yTarget - AvgReadingX
        //  GoTo( AvgReadingY - xTarget ,0, Speed, 0, 14,0,true,true,0.012,true);
        GoTo(-24, yTarget - AvgReadingX, Speed, 0, 14, 0, true, true, 0.012, true);


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
            }

            if (Decel) {
                YL = XL = 0;
            }

        } else if (XDistance == 0) {
            while (Math.abs(Yposition) <= Math.abs(YDistance)) {
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


    private void intakeFunction(double distance, long sleep, double Angle, boolean Beltafter, double speed) {
        ramp.setPosition(0.8);
        sleep(300);
        intake.setPower(-1);
        Conveyor.setPower(-1);
        GoTo(0, distance, speed, Angle, 14, 0, true, true, 0.012, true);

        intake.setPower(1);
        if (Beltafter == false) {
            Conveyor.setPower(0);
        }
        sleep(sleep);
        intake.setPower(0);
        ramp.setPosition(0);
    }

    private void MagFull(double Yl, double Xl, double Timeout, double Distance, double MaintainAngle, double IMUGain) {
        ramp.setPosition(0.8);

        ElapsedTime MagTimer = new ElapsedTime();
        while (MagTimer.seconds() < Timeout) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            CurrentHeading = angles.firstAngle;
            intake.setPower(1);
            LeftLauncher.setPower(-0.75);
            RightLauncher.setPower(-0.75);
            if (rangeSensor.getDistance(DistanceUnit.CM) > Distance) {
                MagTimer.reset();
            }
            MecanumFunction(Yl, Xl, (-MaintainAngle + CurrentHeading) * IMUGain);
        }
        LeftLauncher.setPower(0);
        RightLauncher.setPower(0);
        MecanumFunction(0, 0, 0);
        intake.setPower(-1);
        sleep(250);
        ramp.setPosition(0);
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
            MecanumFunction(0, 0, IMUgain * (-targetangle + CurrentHeading));
        }
        MecanumFunction(0, 0, 0);
    }

    private void BulkCaching() {
        mFr = motor_drive_frAsDcMotor.getCurrentPosition();
        mFl = motor_drive_flAsDcMotor.getCurrentPosition();
        mBr = motor_drive_brAsDcMotor.getCurrentPosition();
        mBl = motor_drive_blAsDcMotor.getCurrentPosition();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        CurrentHeading = angles.firstAngle;
        if (targetVisible) {


        }
    }

    //   while (!(40 - CurrentY <= 1 && 40 - CurrentY >= -1 && -3 - CurrentX <= 1 && -3 - CurrentX >= -1 && (180 - (180 + Zrot)) >= 1 && (180 - (180 + Zrot)) <= -1))
    private void wallTargetTracking(VuforiaLocalizer vufor, Iterable<? extends VuforiaTrackable> allTrackables,
                                    int camDir, double xTarget, double yTarget, double yawTarget, double linTol, double angleTol, double pGain,
                                    long timeOut, boolean TrueTracking, double TargetAngle) {

            /*  I am a function that tracks a wall target using Vuforia using parameters passed on to me
                camDir - Direction that my webcam is pointed.  -1 is to the left, 0 is straight ahead, 1 is to the right
                xTarget - X coordinate target value
                yTarget - Y coord target value
                yawTarget - yaw angle target value
                linTol - XY distance tolerance before deciding we are at target location
                angleTol - angular tolerance before deciding we are at the target location
                pGain - Overall Proportional gain for motor commands based on error
                timeOut - How long to wait before exiting function
             */
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
        while ((!(yTarget - CurrentY <= 0.7 && yTarget - CurrentY >= -0.7 && xTarget - CurrentX <= 0.8 && xTarget - CurrentX >= -0.8))) {
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
/*                        //                   telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
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
  */
                telemetry.update();

//                        Heading = (180 + (180 - rotation.thirdAngle));
                if (camDir == 90) {
                    MecanumFunction(1 * (-0.01 * (xTarget - CurrentX)), (-1 * (0.018 * (yTarget - CurrentY))), TrueTrackSwitch * (-0.0075 * (yawTarget - (CurrentVal))) + (-1 * IMUTrackSwitch * -0.01 * (-TargetAngle + CurrentHeading)));
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
            }
//                    telemetry.update();
            LastVal = CurrentVal;

        }
    }

    private void Initialization() {
        BNO055IMU.Parameters imuParameters;

        Acceleration gravity;
        imuParameters = new BNO055IMU.Parameters();
        TimerA = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        TimerB = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        TimerC = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
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

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        // webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        clarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clarm.setTargetPosition(0);
        clarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        CurrentHeading = angles.firstAngle;
        motor_drive_brAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_drive_frAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_drive_blAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_drive_flAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_drive_flAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_drive_frAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_drive_blAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_drive_brAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_blAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_brAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_blAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_brAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        PIDFCoefficients brpidNew = new PIDFCoefficients(20, 15, 0, 0);
        motor_drive_flAsDcMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        motor_drive_frAsDcMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        motor_drive_blAsDcMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        motor_drive_brAsDcMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
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
            //((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(webcamName, robotFromCamera);
        }

//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // initTfod();
        //webcamName = hardwareMap.get(WebcamName vuforia = ClassFactory.getInstance().createVuforia(parameters);.class, "Webcam 1");
    }

    private void ringScan() {
        QuadRun = 0;
        OneRun = 0;
        NoneRun = 0;
        QuadVote = OneVote = NoneVote = 0;
        TimerC = new ElapsedTime();
        TimerC.reset();
        while (!(isStarted())) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {

                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    telemetry.addData("QuadVotes", QuadVote);
                    telemetry.addData("NoneVotes", NoneVote);
                    telemetry.addData("OneVotes", OneVote);
                    telemetry.update();
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel() == "Quad") {
                            QuadVote += 1;
                        } else if (recognition.getLabel() == "Single") {
                            OneVote += 1;
                        } else {
                            NoneVote += 1;
                        }
                        /*
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                                */

                    }
                    telemetry.update();

                }
            }

        }
        if (QuadVote > OneVote && QuadVote > NoneVote) {
            QuadRun = 1;
        } else if (OneVote > QuadVote && OneVote > NoneVote) {
            OneRun = 1;
        } else if (OneVote < 4 && QuadVote < 4) {
            NoneRun = 1;
        }
        if (tfod != null) {
            tfod.shutdown();

        }
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

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

/*
Liam:
  The maximum power of each motor in the .setPower method is +/- 1.0.  The way MecanumFunction is implemented below
  is not 'correct' in that the calculated value could be as much as +/-3.0 which is not correct.  Please figure out an algorithm
  that will 'scale' the .setPower value to a maximum of 1.0.  This might mean that you will need to adjust the function input
  values so that the resulting setPower value is a maximum of +/-1.0 but keep the inputs keep the same ratio.
 */


    private void MecanumFunction(double YL, double XL, double XR) {
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
            motor_drive_flAsDcMotor.setPower((YL - (-XL - XR)));
            motor_drive_blAsDcMotor.setPower(-(-YL - (-XL + XR)));
            motor_drive_frAsDcMotor.setPower((YL - XL - XR));
            motor_drive_brAsDcMotor.setPower(-(-YL - XL + XR));
        } else {
            motor_drive_flAsDcMotor.setPower((mYL - (-mXL - mXR)));
            motor_drive_blAsDcMotor.setPower(-(-mYL - (-mXL + mXR)));
            motor_drive_frAsDcMotor.setPower((mYL - mXL - mXR));
            motor_drive_brAsDcMotor.setPower(-(-mYL - (-mXL + mXR)));
        }


    }

    private void IMUTurn(double TrgtAngle) {
        double AngleToTurn;

        BulkCaching();
        AngleToTurn = TrgtAngle - CurrentHeading;
        if (AngleToTurn < 0) {
            while (TrgtAngle + 6.5 <= CurrentHeading) {
                BulkCaching();
                if (isStopRequested()) {
                    break;
                }
                if (TrgtAngle - CurrentHeading < -40) {
                    MecanumFunction(0, 0, 0.8);
                } else if (TrgtAngle - CurrentHeading >= -40) {
                    MecanumFunction(0, 0, 0.2);
                }
            }
        } else if (AngleToTurn > 0) {
            while (TrgtAngle - 6.5 >= CurrentHeading) {
                BulkCaching();
                if (isStopRequested()) {
                    break;
                }
                if (TrgtAngle - CurrentHeading > 40) {
                    MecanumFunction(0, 0, -0.8);
                } else if (TrgtAngle - CurrentHeading <= 40) {
                    MecanumFunction(0, 0, -0.2);
                }
            }
        }
        MecanumFunction(0, 0, 0);
    }

    private void AngleAdjustment(double DegreeChange) {
        double AngleToTurn;
        double CurrentAngularPosition;
        BulkCaching();
        CurrentAngularPosition = CurrentHeading;
        AngleToTurn = DegreeChange - CurrentAngularPosition;
        if (AngleToTurn < 0) {
            while (CurrentAngularPosition + DegreeChange < CurrentHeading) {
                BulkCaching();
                if (isStopRequested()) {
                    break;
                }
                MecanumFunction(0, 0, 0.2);
            }
        } else if (AngleToTurn > 0) {
            while (CurrentAngularPosition + DegreeChange > CurrentHeading) {
                BulkCaching();
                if (isStopRequested()) {
                    break;
                }

                MecanumFunction(0, 0, -0.2);
            }
        }
        MecanumFunction(0, 0, 0);
    }

    private void readCurrentHeading() {

    }

    private void DistanceSmoothTravel(double Distance, double Speed, double MaintainAngle,
                                      double IMUGain, boolean Accel_, boolean Decel_, double DecelDistance) {
        ElapsedTime TimerAccel;
        ElapsedTime TimerDecel;
        double AccelDist;
        double DecelDist;
        double ResetTimerAccel_;
        double ResetTimerDecel_;


        motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_blAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_brAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_blAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_brAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Distance = -Distance;
        TimerAccel = new ElapsedTime();
        TimerDecel = new ElapsedTime();
        AccelDist = 240;
        DecelDist = DecelDistance;
        readCurrentHeading();
        EncoderTicks = (Distance / 12.566) * 370;
        ResetTimerAccel_ = 1;
        ResetTimerDecel_ = 1;
        avgEnc = (motor_drive_flAsDcMotor.getCurrentPosition() + motor_drive_frAsDcMotor.getCurrentPosition()) / 2;
        if (Distance > 0) {
            TimerDecel.reset();
            TimerAccel.reset();
            while (avgEnc >= -EncoderTicks && opModeIsActive()) {
                if (isStopRequested()) {
                    break;
                }
                BulkCaching();
                avgEnc = (motor_drive_flAsDcMotor.getCurrentPosition() + motor_drive_frAsDcMotor.getCurrentPosition()) / 2;
                telemetry.addData("Encoder Ticks Target", -EncoderTicks);
                telemetry.addData("FL", motor_drive_flAsDcMotor.getCurrentPosition());
                telemetry.addData("FR", motor_drive_frAsDcMotor.getCurrentPosition());
                telemetry.update();
                if (avgEnc >= -AccelDist && Accel_ == true) {
                    if (ResetTimerAccel_ == 1) {
                        TimerAccel.reset();
                        ResetTimerAccel_ = 0;
                    }
                    MecanumFunction(Math.min(Math.max(Speed * (TimerAccel.seconds() + 1) * 15, -0.2), Speed), 0, (-MaintainAngle + CurrentHeading) * IMUGain);
                } else if (avgEnc >= -EncoderTicks + DecelDist && Decel_ == true) {
                    if (ResetTimerDecel_ == 1) {
                        TimerDecel.reset();
                        ResetTimerDecel_ = 0;
                    }
                    MecanumFunction(Math.min(Math.max(Speed * (0.1 / (0.12 + TimerDecel.seconds())), 0.1), Math.abs(Speed)), 0, (-MaintainAngle + CurrentHeading) * IMUGain);
                } else {
                    MecanumFunction(Speed, 0, (-MaintainAngle + CurrentHeading) * IMUGain);
                }
            }
            TimerAccel.reset();
            TimerDecel.reset();
            motor_drive_flAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor_drive_frAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor_drive_blAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor_drive_brAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            MecanumFunction(0, 0, 0);
        } else {
            TimerDecel.reset();
            TimerAccel.reset();

            motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mFl = motor_drive_flAsDcMotor.getCurrentPosition();
            while (avgEnc <= -EncoderTicks && opModeIsActive()) {
                if (isStopRequested()) {
                    break;
                }
                BulkCaching();
                avgEnc = (motor_drive_flAsDcMotor.getCurrentPosition() + motor_drive_frAsDcMotor.getCurrentPosition()) / 2;
                telemetry.addData("Encoder Ticks Target", -EncoderTicks);
                telemetry.addData("FL", motor_drive_flAsDcMotor.getCurrentPosition());
                telemetry.addData("FR", motor_drive_frAsDcMotor.getCurrentPosition());
                telemetry.update();
                if (avgEnc >= -AccelDist && Accel_ == true) {
                    if (ResetTimerAccel_ == 1) {
                        TimerAccel.reset();
                        ResetTimerAccel_ = 0;
                    }
                    MecanumFunction(Math.min(Math.max(-Speed * TimerAccel.seconds() * 1, -Speed), -0.1), 0, (-MaintainAngle + CurrentHeading) * IMUGain);
                } else if (avgEnc <= -EncoderTicks - DecelDist && Decel_ == true) {
                    if (ResetTimerDecel_ == 1) {
                        TimerDecel.reset();
                        ResetTimerDecel_ = 0;
                    } else if (avgEnc >= -EncoderTicks) {
                        break;
                    }
                    MecanumFunction(Math.min(Math.max(-Speed * (0.1 / (0.08 + TimerDecel.seconds())), -Speed), -0.1), 0, (-MaintainAngle + CurrentHeading) * IMUGain);
                } else {
                    MecanumFunction(-Speed, 0, (-MaintainAngle + CurrentHeading) * IMUGain);
                }
            }
            TimerAccel.reset();
            TimerDecel.reset();
            if (Decel_ == true) {
                motor_drive_flAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor_drive_frAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor_drive_blAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor_drive_brAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                MecanumFunction(0, 0, 0);

            } else {
                motor_drive_flAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motor_drive_frAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motor_drive_blAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motor_drive_brAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            }

        }
        MecanumFunction(0, 0, 0);
    }

    private void DistanceStrafe(double Distance, double Speed, double MaintainAngle,
                                double IMUGain, boolean Accel_, boolean Decel_, double DecelDistance) {
        ElapsedTime TimerAccel;
        ElapsedTime TimerDecel;
        double AccelDist;
        double DecelDist;
        double ResetTimerAccel_;
        double ResetTimerDecel_;


        motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_blAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_brAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_blAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_brAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Distance = -Distance;
        TimerAccel = new ElapsedTime();
        TimerDecel = new ElapsedTime();
        AccelDist = 240;
        DecelDist = DecelDistance;
        readCurrentHeading();
        EncoderTicks = (Distance / 12.566) * 370;
        ResetTimerAccel_ = 1;
        ResetTimerDecel_ = 1;
        avgEnc = (motor_drive_flAsDcMotor.getCurrentPosition() + motor_drive_brAsDcMotor.getCurrentPosition()) / 2;
        if (Distance > 0) {
            TimerDecel.reset();
            TimerAccel.reset();
            while (avgEnc >= -EncoderTicks && opModeIsActive()) {
                BulkCaching();
                avgEnc = (motor_drive_flAsDcMotor.getCurrentPosition() + motor_drive_brAsDcMotor.getCurrentPosition()) / 2;
                telemetry.addData("Encoder Ticks Target", -EncoderTicks);
                telemetry.addData("FL", motor_drive_flAsDcMotor.getCurrentPosition());
                telemetry.addData("BR", motor_drive_brAsDcMotor.getCurrentPosition());
                telemetry.update();
                if (avgEnc >= -AccelDist && Accel_ == true) {
                    if (ResetTimerAccel_ == 1) {
                        TimerAccel.reset();
                        ResetTimerAccel_ = 0;
                    }
                    MecanumFunction(Math.min(Math.max(Speed * (TimerAccel.seconds() + 1) * 15, -0.2), Speed), 0, (-MaintainAngle + CurrentHeading) * IMUGain);
                } else if (avgEnc >= -EncoderTicks + DecelDist && Decel_ == true) {
                    if (ResetTimerDecel_ == 1) {
                        TimerDecel.reset();
                        ResetTimerDecel_ = 0;
                    }
                    MecanumFunction(Math.min(Math.max(Speed * (0.1 / (0.06 + TimerDecel.seconds())), 0.1), Math.abs(Speed)), 0, (-MaintainAngle + CurrentHeading) * IMUGain);
                } else {
                    MecanumFunction(Speed, 0, (-MaintainAngle + CurrentHeading) * IMUGain);
                }
            }
            TimerAccel.reset();
            TimerDecel.reset();
            motor_drive_flAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor_drive_frAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor_drive_blAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor_drive_brAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            MecanumFunction(0, 0, 0);
        } else {
            TimerDecel.reset();
            TimerAccel.reset();

            motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mFl = motor_drive_flAsDcMotor.getCurrentPosition();
            while (avgEnc <= -EncoderTicks && opModeIsActive()) {
                BulkCaching();
                avgEnc = (motor_drive_flAsDcMotor.getCurrentPosition() + motor_drive_brAsDcMotor.getCurrentPosition()) / 2;
                telemetry.addData("Encoder Ticks Target", -EncoderTicks);
                telemetry.addData("FL", motor_drive_flAsDcMotor.getCurrentPosition());
                telemetry.addData("BR", motor_drive_brAsDcMotor.getCurrentPosition());
                telemetry.update();
                if (avgEnc >= -AccelDist && Accel_ == true) {
                    if (ResetTimerAccel_ == 1) {
                        TimerAccel.reset();
                        ResetTimerAccel_ = 0;
                    }
                    MecanumFunction(Math.min(Math.max(-Speed * TimerAccel.seconds() * 1, -Speed), -0.1), 0, (-MaintainAngle + CurrentHeading) * IMUGain);
                } else if (avgEnc <= -EncoderTicks - DecelDist && Decel_ == true) {
                    if (ResetTimerDecel_ == 1) {
                        TimerDecel.reset();
                        ResetTimerDecel_ = 0;
                    } else if (avgEnc >= -EncoderTicks) {
                        break;
                    }
                    MecanumFunction(Math.min(Math.max(-Speed * (0.1 / (0.6 + TimerDecel.seconds())), -Speed), -0.1), 0, (-MaintainAngle + CurrentHeading) * IMUGain);
                } else {
                    MecanumFunction(-Speed, 0, (-MaintainAngle + CurrentHeading) * IMUGain);
                }
            }
            TimerAccel.reset();
            TimerDecel.reset();
            if (Decel_ == true) {
                motor_drive_flAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor_drive_frAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor_drive_blAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor_drive_brAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                MecanumFunction(0, 0, 0);

            } else {
                motor_drive_flAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motor_drive_frAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motor_drive_blAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motor_drive_brAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            }

        }
    }

    private void GoTo(double XDistance, double YDistance, double Speed, double MaintainAngle, double AccelDistance, double DecelDistance, boolean Accel, boolean Decel, double IMUGain, boolean Reset) {

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
                MecanumFunction(Math.min(Math.max(Speed * ((Math.abs(Yposition) * 0.6) / Accelerate) * (((Math.abs(YDistance) - Math.abs(Yposition)) * 0.4) / Decelerate), 0.025), YSpeed) * Y * YD, Math.min(Math.max(Speed * ((Math.abs(Xposition) * 1) / XAccelerate) * (((Math.abs(XDistance) - Math.abs(Xposition)) * 0.7) / XDecelerate), 0.49), Speed) * X * XD, (-MaintainAngle + CurrentHeading) * IMUGain);


                YSpeed = Math.abs(Speed * (Math.abs(YDistance) / Math.sqrt(((YDistance * YDistance) + (XDistance * XDistance)))) * Y);
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
                if (Decel && ((Math.abs(YDistance) - Math.abs(Yposition) < 18))) {
                    if (Math.abs(YDistance) - Math.abs(Yposition) < 18) {
                        //  telemetry.addData("Decel?", 1);
                        Decelerate = 1;
                        // telemetry.update();

                    }
                } else {
                    Decelerate = ((Math.abs(YDistance) - Math.abs(Yposition)) * 0.4);
                }

                MecanumFunction(Math.min(Math.max(Speed * ((Math.abs(Yposition) * 0.6) / Accelerate) * (((Math.abs(YDistance) - Math.abs(Yposition)) * 0.1) / Decelerate), 0.025), Speed) * Y * YD, 0, (-MaintainAngle + CurrentHeading) * IMUGain);

            }

            if (Decel) {
                MecanumFunction(0, 0, 0);
            }


        }

    }

    private void PersonalityStateMachine() {
        switch (CurrentState) {
            case MOVE_RIGHT:
                if (gamepad1.right_stick_x > 0 || gamepad1.left_stick_x > 0) {
                    telemetry.addData("strafe and turn right", CurrentState);
                    telemetry.update();
                } else if (gamepad1.right_stick_x < 0 || gamepad1.left_stick_x < 0) {
                    CurrentState = State.MOVE_LEFT;
                } else if (Yposition > 0) {
                    CurrentState = State.MOVE_FORWARD;
                } else if (Yposition < 0) {
                    CurrentState = State.MOVE_BACKWARDS;
                } else if (RightLauncher.getPower() < 0) {
                    CurrentState = State.SHOOT;
                } else if (intake.getPower() > 0) {
                    CurrentState = State.INTAKE;
                } else {
                    CurrentState = State.IDLE;
                }
                break;

            case MOVE_LEFT:
                if (gamepad1.right_stick_x < 0 || gamepad1.left_stick_x < 0) {
                    telemetry.addData("strafe or turn", CurrentState);
                    telemetry.update();
                } else if (gamepad1.right_stick_x > 0 || gamepad1.left_stick_x > 0) {
                    CurrentState = State.MOVE_RIGHT;
                } else if (Yposition > 0) {
                    CurrentState = State.MOVE_FORWARD;
                } else if (Yposition < 0) {
                    CurrentState = State.MOVE_BACKWARDS;
                } else if (RightLauncher.getPower() < 0) {
                    CurrentState = State.SHOOT;
                } else if (intake.getPower() > 0) {
                    CurrentState = State.INTAKE;
                } else {
                    CurrentState = State.IDLE;
                }
                break;

            case MOVE_FORWARD:
                if (Yposition > 0) {
                    telemetry.addData("move forward", CurrentState);
                    telemetry.update();
                } else if (gamepad1.right_stick_x > 0 || gamepad1.left_stick_x > 0) {
                    CurrentState = State.MOVE_RIGHT;
                } else if (gamepad1.right_stick_x < 0 || gamepad1.left_stick_x < 0) {
                    CurrentState = State.MOVE_LEFT;
                } else if (Yposition < 0) {
                    CurrentState = State.MOVE_BACKWARDS;
                } else if (RightLauncher.getPower() < 0) {
                    CurrentState = State.SHOOT;
                } else if (intake.getPower() > 0) {
                    CurrentState = State.INTAKE;
                } else {
                    CurrentState = State.IDLE;
                }
                break;

            case MOVE_BACKWARDS:
                if (Yposition < 0) {
                    telemetry.addData("move backwards", CurrentState);
                    telemetry.update();
                } else if (gamepad1.right_stick_x > 0 || gamepad1.left_stick_x > 0) {
                    CurrentState = State.MOVE_RIGHT;
                } else if (gamepad1.right_stick_x < 0 || gamepad1.left_stick_x < 0) {
                    CurrentState = State.MOVE_LEFT;
                } else if (Yposition > 0) {
                    CurrentState = State.MOVE_FORWARD;
                } else if (RightLauncher.getPower() < 0) {
                    CurrentState = State.SHOOT;
                } else if (intake.getPower() > 0) {
                    CurrentState = State.INTAKE;
                } else {
                    CurrentState = State.IDLE;
                }
                break;

            case SHOOT:
                if (RightLauncher.getPower() < 0) {
                    telemetry.addData("shoot", CurrentState);
                    telemetry.update();
                } else if (gamepad1.right_stick_x > 0 || gamepad1.left_stick_x > 0) {
                    CurrentState = State.MOVE_RIGHT;
                } else if (gamepad1.right_stick_x < 0 || gamepad1.left_stick_x < 0) {
                    CurrentState = State.MOVE_LEFT;
                } else if (Yposition > 0) {
                    CurrentState = State.MOVE_FORWARD;
                } else if (Yposition < 0) {
                    CurrentState = State.MOVE_BACKWARDS;
                } else if (intake.getPower() > 0) {
                    CurrentState = State.INTAKE;
                } else {
                    CurrentState = State.IDLE;
                }
                break;

            case INTAKE:
                if (intake.getPower() > 0) {
                    telemetry.addData("intake", CurrentState);
                    telemetry.update();
                } else if (gamepad1.right_stick_x > 0 || gamepad1.left_stick_x > 0) {
                    CurrentState = State.MOVE_RIGHT;
                } else if (gamepad1.right_stick_x < 0 || gamepad1.left_stick_x < 0) {
                    CurrentState = State.MOVE_LEFT;
                } else if (Yposition > 0) {
                    CurrentState = State.MOVE_FORWARD;
                } else if (Yposition < 0) {
                    CurrentState = State.MOVE_BACKWARDS;
                } else if (RightLauncher.getPower() < 0) {
                    CurrentState = State.SHOOT;
                } else {
                    CurrentState = State.IDLE;
                }
                break;

            case IDLE:
                if ((RightLauncher.getPower() == 0) && (intake.getPower() == 0) && (gamepad1.right_stick_x == 0) && (Yposition == 0) && (gamepad1.left_stick_x == 0)) {
                    telemetry.addData("idle", CurrentState);
                    telemetry.update();
                } else if (gamepad1.right_stick_x > 0 && gamepad1.left_stick_x > 0) {
                    CurrentState = State.MOVE_RIGHT;
                } else if (gamepad1.right_stick_x < 0 && gamepad1.left_stick_x < 0) {
                    CurrentState = State.MOVE_LEFT;
                } else if (Yposition > 0) {
                    CurrentState = State.MOVE_FORWARD;
                } else if (Yposition < 0) {
                    CurrentState = State.MOVE_BACKWARDS;
                } else if (RightLauncher.getPower() < 0) {
                    CurrentState = State.SHOOT;
                } else {
                    CurrentState = State.INTAKE;
                }
                break;

        }
    }
}

