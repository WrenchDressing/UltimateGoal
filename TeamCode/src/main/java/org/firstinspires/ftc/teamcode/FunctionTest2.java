package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

@Autonomous(name = "FunctionTest2", group = "")
@Disabled
public class FunctionTest2 extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private DcMotor RightLauncher;
    private DcMotor LeftLauncher;
    private CRServo Conveyor;
    public static final double NEW_P = 25;
    public static final double NEW_I = 18;
    public static final double NEW_D = 0;
    public static final double NEW_F = 0;
    private DcMotor clarm;
    private DcMotor intake;
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
    WebcamName webcamName = null;
    private DcMotorEx motor_drive_flAsDcMotor, motor_drive_blAsDcMotor, motor_drive_brAsDcMotor, motor_drive_frAsDcMotor;
    private BNO055IMU imu;
    private VuforiaCurrentGame vuforiaUltimateGoal;
    Orientation angles;
    ElapsedTime TimerA;
    ElapsedTime TimerC;
    float CurrentHeading;
    ElapsedTime TimerB;
    double CurrentVal;
    double LastVal;
    double QuadRun;
    double OneRun;
    double NoneRun;
    double EncoderTicks;
    double mFr, mFl, mBl, mBr;
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
    boolean frOverload, flOverload, blOverload, brOverload;
    private Servo camServo;


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
        clarm = hardwareMap.get(DcMotor.class, "clarm");
        claw = hardwareMap.get(Servo.class, "claw");
        ramp = hardwareMap.get(Servo.class, "ramp");
        intake = hardwareMap.dcMotor.get("intake");
        PIDFCoefficients pidOrig = motor_drive_flAsDcMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

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
        final float CAMERA_FORWARD_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
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
        }
        waitForStart();

        Conveyor.setPower(-0.6);
        targetsUltimateGoal.activate();
        camServo.setPosition(0.79);
        DistanceSmoothTravel(22, .4, 0, 0.025, true, true, 1400);
        ramp.setPosition(0.09);
        ringScan();
        camServo.setPosition(.5);
        DistanceSmoothTravel(42, .3, 0, 0.025, true, true, 4000);

        AngularAdjustment(0, 0.01);
        Shoot(-0.31, 0.745, 1.5, false);
        AngularAdjustment(-4, 0.019);
        Shoot(-0.31, 0.73, 1.5, false);
        AngularAdjustment(-8.5, 0.025);
        Shoot(-0.31, 0.733, 1.5, true);
        IMUTurn(0);

        clarm.setTargetPosition(700);
        clarm.setPower(0.6);

        if (QuadRun == 1) {
            DistanceSmoothTravel(38, .6, 33, 0.025, true, true, 1400);
            IMUTurn(90);
            DistanceSmoothTravel(15, .4, 90, 0.025, true, true, 1400);
            clarm.setTargetPosition(850);
            clarm.setPower(0.15);
            sleep(250);
            claw.setPosition(0.35);
            sleep(400);
            clarm.setTargetPosition(450);
            clarm.setPower(-0.6);
            sleep(200);
            IMUTurn(13);
            DistanceSmoothTravel(-22, .7, 13, 0.025, true, true, 1400);
            wallTargetTracking(vuforia, allTrackables, 90, 0, 58, 0, 10, 2, 1, 3, false, 0);
            DistanceSmoothTravel(-18, .5, 0, 0.025, true, true, 2800);
            IMUTurn(-64);
            clarm.setTargetPosition(950);
            clarm.setPower(0.5);
            sleep(750);
            claw.setPosition(0.03);
            sleep(500);
            clarm.setTargetPosition(450);
            clarm.setPower(-0.7);
            IMUTurn(0);
            DistanceSmoothTravel(54, 0.7, 0, 0.025, true, true, 1400);
            IMUTurn(90);
            DistanceSmoothTravel(14, .4, 90, 0.025, true, true, 1400);
            IMUTurn(120);
            clarm.setTargetPosition(850);
            clarm.setPower(0.15);
            sleep(300);
            claw.setPosition(0.35);
            sleep(300);
            clarm.setTargetPosition(-5);
            clarm.setPower(-0.6);
            sleep(400);
            IMUTurn(0);
            DistanceSmoothTravel(-19, .7, 0, 0.025, true, true, 1400);
            MecanumFunction(0, 0, 0);


        } else if (OneRun == 1) {
            DistanceSmoothTravel(8, .4, 12.5, 0.025, true, true, 1400);
            IMUTurn(90);
            DistanceSmoothTravel(13, .4, 90, 0.025, true, true, 1400);
            clarm.setTargetPosition(850);
            clarm.setPower(0.15);
            sleep(500);
            claw.setPosition(0.35);
            sleep(500);
            clarm.setTargetPosition(450);
            clarm.setPower(-0.6);
            sleep(750);
            DistanceSmoothTravel(3, .4, 90, 0.025, true, true, 1400);
            IMUTurn(-5);
            DistanceSmoothTravel(-4, .4, -5, 0.025, true, true, 1400);
            wallTargetTracking(vuforia, allTrackables, 90, 0, 58, 0, 10, 2, 1, 3, false, 0);
            DistanceSmoothTravel(-17.7, .4, 0, 0.025, true, true, 1400);
            IMUTurn(-68);
            clarm.setTargetPosition(950);
            clarm.setPower(0.5);
            sleep(400);
            claw.setPosition(0.03);
            sleep(500);
            clarm.setTargetPosition(450);
            clarm.setPower(-0.7);
            IMUTurn(0);
            DistanceSmoothTravel(60, .4, 7, 0.025, true, true, 1400);
            clarm.setTargetPosition(850);
            clarm.setPower(0.7);
            sleep(1000);
            claw.setPosition(0.35);
            sleep(900);
            clarm.setTargetPosition(-5);
            clarm.setPower(-0.7);
            sleep(1000);
            DistanceSmoothTravel(-16, .4, 0, 0.025, true, true, 1400);


        } else if (NoneRun == 1) {
            IMUTurn(0);
            DistanceSmoothTravel(-6, .4, 0, 0.025, true, true, 1400);
            sleep(200);
            IMUTurn(90);
            DistanceSmoothTravel(36, .4, 90, 0.025, true, true, 1400);
            clarm.setTargetPosition(850);
            clarm.setPower(0.15);
            sleep(500);
            claw.setPosition(0.35);
            sleep(500);
            clarm.setTargetPosition(450);
            clarm.setPower(-0.63);
            sleep(500);
            camServo.setPosition(0.6);
            DistanceSmoothTravel(-6, .4, 90, 0.025, true, true, 1400);
            IMUTurn(-5);
            DistanceSmoothTravel(2, .4, -5, 0.025, true, true, 1400);
            wallTargetTracking(vuforia, allTrackables, 90, -8.4, 60, 0, 10, 2, 1, 3, false, 0);
            camServo.setPosition(0.5);
            DistanceSmoothTravel(-8.25, .4, 0, 0.025, true, true, 1400);
            IMUTurn(-55.5);
            clarm.setTargetPosition(1000);
            clarm.setPower(0.5);
            sleep(400);
            claw.setPosition(0.03);
            sleep(500);
            clarm.setTargetPosition(450);
            clarm.setPower(-0.7);
            IMUTurn(0);
            DistanceSmoothTravel(10, .4, 0, 0.025, true, true, 1400);
            IMUTurn(90);
            DistanceSmoothTravel(10, .4, 90, 0.025, true, true, 1400);
            clarm.setTargetPosition(850);
            clarm.setPower(0.15);
            sleep(300);
            claw.setPosition(0.35);
            sleep(300);
            clarm.setTargetPosition(-5);
            clarm.setPower(-0.6);
            sleep(600);
            DistanceSmoothTravel(-26, .4, 90, 0.025, true, true, 1400);
            IMUTurn(0);
            DistanceSmoothTravel(18, .4, 0, 0.025, true, true, 1400);


            //DistanceSmoothTravel(4, .6, 0, 0.1, true, true, 1400);
        }
        clarm.setTargetPosition(0);
        clarm.setPower(0.4);
        sleep(1500);
        //  targetsUltimateGoal.activate();


        // wallTargetTracking(vuforia, allTrackables, 90, -3, 40.0, 0, 10, 2, 1, 70000, false, 5);
    }

    private void AngularAdjustment(double targetangle, double IMUgain) {
        while ((!((-targetangle + CurrentHeading) <= 0.25 && (-targetangle + CurrentHeading) >= -0.25))) {
            BulkCaching();
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
        while ((!(yTarget - CurrentY <= 1 && yTarget - CurrentY >= -1 && xTarget - CurrentX <= 1 && xTarget - CurrentX >= -1))) {
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
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

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
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        RightLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_blAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_brAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        PIDFCoefficients brpidNew = new PIDFCoefficients(35, 25, 0, 0);
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
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(webcamName, robotFromCamera);
        }

//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // initTfod();
        //webcamName = hardwareMap.get(WebcamName vuforia = ClassFactory.getInstance().createVuforia(parameters);.class, "Webcam 1");
    }

    private void ringScan() {
        QuadRun = 0;
        OneRun = 0;
        NoneRun = 0;
        TimerC = new ElapsedTime();
        TimerC.reset();
        while ((TimerC.seconds() < 1)) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                tfod.setZoom(2.5, 1.78);
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {

                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel() == "Quad") {
                            QuadRun = 1;
                            break;
                        }
                        if (recognition.getLabel() == "Single") {
                            OneRun = 1;
                            break;
                        }
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                    }
                    telemetry.update();

                }
            }

        }
        if (tfod != null) {
            tfod.shutdown();
            if (QuadRun == 0 && OneRun == 0) {
                NoneRun = 1;
            }

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
        tfodParameters.minResultConfidence = 0.8f;
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
            motor_drive_flAsDcMotor.setPower((-YL - (-XL - XR)));
            motor_drive_blAsDcMotor.setPower((YL - (-XL + XR)));
            motor_drive_frAsDcMotor.setPower((-YL + XL - XR));
            motor_drive_brAsDcMotor.setPower((YL + (XL + XR)));
        } else {
            motor_drive_flAsDcMotor.setPower((-mYL - (-mXL - mXR)));
            motor_drive_blAsDcMotor.setPower((mYL - (-mXL + mXR)));
            motor_drive_frAsDcMotor.setPower((-mYL + mXL - mXR));
            motor_drive_brAsDcMotor.setPower((mYL + (mXL + mXR)));
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

    /*private void RingPickUp (double ringNumber, double timer) {
        if intake.setVelocity
    }*/

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
            motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_drive_frAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_drive_flAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mFl = motor_drive_flAsDcMotor.getCurrentPosition();
            while (avgEnc <= -EncoderTicks && opModeIsActive()) {
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
                    MecanumFunction(Math.min(Math.max(-Speed * (0.1 / (0.1 + TimerDecel.seconds())), -Speed), -0.1), 0, (-MaintainAngle + CurrentHeading) * IMUGain);
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
}
