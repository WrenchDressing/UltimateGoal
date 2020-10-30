package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.TrackableResult;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.Base64;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;


import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.ArrayList;
import java.util.List;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
@Autonomous(name = "FunctionTest", group = "")

public class FunctionTest extends LinearOpMode {

    private static final String VUFORIA_KEY =
            "Ae2mEyz/////AAABmQBmoTE94ki5quwzTT/OlIIeOueUfjuHL/5k1VNWN943meU2RmiXCJ9eX3rUR/2CkwguvbBU45e1SzrbTAwz3ZzJXc7XN1ObKk/7yPHQeulWpyJgpeZx+EqmZW6VE6yG4mNI1mshKI7vOgOtYxqdR8Yf7YwBPd4Ruy3NVK01BwBl1F8V/ndY26skaSlnWqpibCR3XIvVG0LXHTdNn/ftZyAFmCedLgLi1UtNhr2eXZdr6ioikyRYEe7qsWZPlnwVn5DaQoTcgccZV4bR1/PEvDLn7jn1YNwSimTC8glK+5gnNpO+X7BiZa5LcqtYEpvk/QNQda0Fd+wHQDXA8ojeMUagawtkQGJvpPpz9c6p4fad";
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    WebcamName webcamName = null;
    private DcMotorEx motor_drive_flAsDcMotor, motor_drive_blAsDcMotor, motor_drive_brAsDcMotor, motor_drive_frAsDcMotor;
    private BNO055IMU imu;
    private VuforiaCurrentGame vuforiaUltimateGoal;
    Orientation angles;
    ElapsedTime TimerA;
    float CurrentHeading;
    ElapsedTime TimerB;
    double EncoderTicks;
    double mFr, mFl, mBl, mBr;
    double CurrentX, CurrentY;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;


    @Override
    public void runOpMode() {
        vuforiaUltimateGoal = new VuforiaCurrentGame();
        motor_drive_flAsDcMotor = hardwareMap.get(DcMotorEx.class, "motor_drive_flAsDcMotor");
        motor_drive_frAsDcMotor = hardwareMap.get(DcMotorEx.class, "motor_drive_frAsDcMotor");
        motor_drive_blAsDcMotor = hardwareMap.get(DcMotorEx.class, "motor_drive_blAsDcMotor");
        motor_drive_brAsDcMotor = hardwareMap.get(DcMotorEx.class, "motor_drive_brAsDcMotor");
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        Initialization();
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
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
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        waitForStart();
        targetsUltimateGoal.activate();
        if (opModeIsActive()) {
            targetVisible = false;
            while (opModeIsActive()) {
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        targetVisible = true;
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                    }
                }
                WallTargetTracking();
                //DistanceSmoothTravel(20, 0.4, 0, 0.025, true, true, 500);
                // DistanceSmoothTravel(20, 0.4, 0, 0.025, true, true, 500);
            }
            }
        }


        private void BulkCaching () {
            mFr = motor_drive_frAsDcMotor.getCurrentPosition();
            mFl = motor_drive_flAsDcMotor.getCurrentPosition();
            mBr = motor_drive_brAsDcMotor.getCurrentPosition();
            mBl = motor_drive_blAsDcMotor.getCurrentPosition();

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            CurrentHeading = angles.firstAngle;
            if (targetVisible) {
                VectorF translation = lastLocation.getTranslation();
                CurrentY = translation.get(1) / mmPerInch;
                CurrentX = translation.get(0) / mmPerInch;
            }
        }

        private void WallTargetTracking () {

                BulkCaching();
                if (targetVisible) {

                    // express position (translation) of robot in inches.
                    VectorF translation = lastLocation.getTranslation();
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                    telemetry.addData("Motorvalue", (-0.0375 * 40 - (translation.get(1) / mmPerInch)));
                    // express the rotation of the robot in degrees.

                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                    MecanumFunction(1 * (-0.0375 * (40 - CurrentY)), (1 * (-0.0375 * (-3 - CurrentX))), 0.0003 * (180 - (180 + rotation.thirdAngle)));
                } else {
                    telemetry.addData("Visible Target", "none");
                }
                telemetry.update();
            }


    private void WallTargetTrackingBlueGoal () {
      /*
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }
        */
        if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                telemetry.addData("Motorvalue", (-0.0375 * 40 - (translation.get(1) / mmPerInch)));
                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                MecanumFunction(1 * (-0.0375 * (41.9 - (translation.get(1) / mmPerInch))), (0 * (-0.0375 * (0 - translation.get(0) / mmPerInch))), 0.000 * (89 - (180 + rotation.thirdAngle)));
        } else {
            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();
    }

        private void Initialization () {
            BNO055IMU.Parameters imuParameters;
            Acceleration gravity;
            imuParameters = new BNO055IMU.Parameters();
            TimerA = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            TimerB = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            imuParameters.loggingEnabled = false;
            imu.initialize(imuParameters);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = imu.getGravity();
            CurrentHeading = angles.firstAngle;
            motor_drive_brAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor_drive_frAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            motor_drive_blAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            motor_drive_flAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor_drive_flAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor_drive_frAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor_drive_blAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor_drive_brAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_drive_blAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_drive_brAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_drive_blAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_drive_brAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        private void readCurrentHeading () {
        }

        private void MecanumFunction ( double YL, double XL, double XR){
            motor_drive_flAsDcMotor.setPower(-YL - (XL - XR));
            motor_drive_blAsDcMotor.setPower(YL - (XL + XR));
            motor_drive_frAsDcMotor.setPower(-YL + XL - XR);
            motor_drive_brAsDcMotor.setPower(YL + (XL + XR));
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
                    MecanumFunction(0, 0, 1);
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
                    MecanumFunction(0, 0, -1);
                } else if (TrgtAngle - CurrentHeading <= 40) {
                    MecanumFunction(0, 0, -0.2);
                }
            }
        }
        MecanumFunction(0, 0, 0);
    }
        private void DistanceSmoothTravel ( double Distance, double Speed, double MaintainAngle,
        double IMUGain, boolean Accel_, boolean Decel_, double DecelDistance){
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
            EncoderTicks = (Distance / 12.566) * 480;
            ResetTimerAccel_ = 1;
            ResetTimerDecel_ = 1;
            if (Distance > 0) {
                TimerDecel.reset();
                TimerAccel.reset();
                while (mFl >= -EncoderTicks && opModeIsActive()) {
                    BulkCaching();
                    telemetry.addData("Loop", 1);
                    telemetry.addData("Encoder Ticks Target", EncoderTicks);
                    telemetry.addData("mFl input", mFl);
                    telemetry.update();
                    if (mFl <= AccelDist && Accel_ == true) {
                        if (ResetTimerAccel_ == 1) {
                            TimerAccel.reset();
                            ResetTimerAccel_ = 0;
                        }
                        MecanumFunction(Math.min(Math.max(Speed * (TimerAccel.seconds() + 1) * 15, 0.2), Math.abs(Speed)), 0, (-MaintainAngle + CurrentHeading) * IMUGain);
                    } else if (mFl >= EncoderTicks - DecelDist && Decel_ == true) {
                        if (ResetTimerDecel_ == 1) {
                            TimerDecel.reset();
                            ResetTimerDecel_ = 0;
                        }
                        MecanumFunction(Math.min(Math.max(Speed * (0.1 / (0.1 + TimerDecel.seconds())), 0.1), Math.abs(Speed)), 0, (-MaintainAngle + CurrentHeading) * IMUGain);
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
                motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                mFl = motor_drive_flAsDcMotor.getCurrentPosition();
                while (mFl <= -EncoderTicks && opModeIsActive()) {
                    BulkCaching();
                    telemetry.addData("Encoder Ticks Target", -EncoderTicks);
                    telemetry.addData("Loop", 2);
                    telemetry.addData("mFl input", mFl);
                    telemetry.update();
                    if (mFl >= -AccelDist && Accel_ == true) {
                        if (ResetTimerAccel_ == 1) {
                            TimerAccel.reset();
                            ResetTimerAccel_ = 0;
                        }
                        MecanumFunction(Math.min(Math.max(-Speed * TimerAccel.seconds() * 1, -Speed), -0.1), 0, (-MaintainAngle + CurrentHeading) * IMUGain);
                    } else if (mFl <= -EncoderTicks - DecelDist && Decel_ == true) {
                        if (ResetTimerDecel_ == 1) {
                            TimerDecel.reset();
                            ResetTimerDecel_ = 0;
                        } else if (mFl >= -EncoderTicks){
                            break;
                        }
                        MecanumFunction(Math.min(Math.max(-Speed * (0.1 / (0.1 + TimerDecel.seconds())), -Speed), -0.1), 0, (-MaintainAngle + CurrentHeading) * IMUGain);
                    } else {
                        MecanumFunction(-Speed, 0, (-MaintainAngle + CurrentHeading) * IMUGain);
                    }
                }
                TimerAccel.reset();
                TimerDecel.reset();
                motor_drive_flAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor_drive_frAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor_drive_blAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor_drive_brAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                MecanumFunction(0, 0, 0);
            }
        }
    }


