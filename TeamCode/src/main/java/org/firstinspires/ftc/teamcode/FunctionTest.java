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


import org.firstinspires.ftc.robotcore.external.ClassFactory;
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
    private DcMotor motor_drive_flAsDcMotor;
    private DcMotor motor_drive_frAsDcMotor;
    private DcMotor motor_drive_blAsDcMotor;
    private DcMotor motor_drive_brAsDcMotor;
    private BNO055IMU imu;
    private VuforiaCurrentGame vuforiaUltimateGoal;
    Orientation angles;
    ElapsedTime TimerA;
    float CurrentHeading;
    ElapsedTime TimerB;
    double EncoderTicks;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;


    @Override
    public void runOpMode() {
        vuforiaUltimateGoal = new VuforiaCurrentGame();
        motor_drive_flAsDcMotor = hardwareMap.dcMotor.get("motor_drive_flAsDcMotor");
        motor_drive_frAsDcMotor = hardwareMap.dcMotor.get("motor_drive_frAsDcMotor");
        motor_drive_blAsDcMotor = hardwareMap.dcMotor.get("motor_drive_blAsDcMotor");
        motor_drive_brAsDcMotor = hardwareMap.dcMotor.get("motor_drive_brAsDcMotor");
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
            }
        }
    }

        private void WallTargetTracking () {
                if (targetVisible) {
                    // express position (translation) of robot in inches.
                    VectorF translation = lastLocation.getTranslation();
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                    telemetry.addData("value", -0.0002*(translation.get(1)));
                    MecanumFunction( Math.min(Math.max(-0.0002 * (-10 - translation.get(1)), -1), 1),(Math.min(Math.max(0 * (6.2 - translation.get(0)), -0.5), 0.5)), Math.min(Math.max(0 * (0 - translation.get(2)), -0.2), 0.2));

                    // express the rotation of the robot in degrees.
                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
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
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            CurrentHeading = angles.firstAngle;
        }
        private void MecanumFunction ( double YL, double XL, double XR){
            motor_drive_flAsDcMotor.setPower(YL - (XL + XR));
            motor_drive_blAsDcMotor.setPower(YL - (XL - XR));
            motor_drive_frAsDcMotor.setPower(YL + XL + XR);
            motor_drive_brAsDcMotor.setPower(YL + (XL - XR));
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
                while (motor_drive_flAsDcMotor.getCurrentPosition() <= EncoderTicks) {
                    readCurrentHeading();
                    if (motor_drive_flAsDcMotor.getCurrentPosition() <= AccelDist && Accel_ == true) {
                        if (ResetTimerAccel_ == 1) {
                            TimerAccel.reset();
                            ResetTimerAccel_ = 0;
                        }
                        MecanumFunction(Math.min(Math.max(Speed * (TimerAccel.seconds() + 1) * 15, 0.2), Math.abs(Speed)), 0, (-MaintainAngle + CurrentHeading) * IMUGain);
                    } else if (motor_drive_flAsDcMotor.getCurrentPosition() >= EncoderTicks - DecelDist && Decel_ == true) {
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
                while (motor_drive_flAsDcMotor.getCurrentPosition() >= EncoderTicks) {
                    readCurrentHeading();
                    if (motor_drive_flAsDcMotor.getCurrentPosition() >= -AccelDist && Accel_ == true) {
                        if (ResetTimerAccel_ == 1) {
                            TimerAccel.reset();
                            ResetTimerAccel_ = 0;
                        }
                        MecanumFunction(Math.min(Math.max(-Speed * TimerAccel.seconds() * 1, -Speed), -0.1), 0, (-MaintainAngle + CurrentHeading) * IMUGain);
                    } else if (motor_drive_flAsDcMotor.getCurrentPosition() <= EncoderTicks + DecelDist && Decel_ == true) {
                        if (ResetTimerDecel_ == 1) {
                            TimerDecel.reset();
                            ResetTimerDecel_ = 0;
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


