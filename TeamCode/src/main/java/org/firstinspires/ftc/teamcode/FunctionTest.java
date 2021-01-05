//test
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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

@Autonomous(name = "FunctionTest", group = "")

public class FunctionTest extends LinearOpMode {
    public static final double NEW_P = 30;
    public static final double NEW_I = 17;
    public static final double NEW_D = 0;
    public static final double NEW_F = 0;
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
    double CurrentVal;
    double LastVal;
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

    @Override
    public void runOpMode() {


        // vuforiaUltimateGoal = new VuforiaCurrentGame();
        motor_drive_flAsDcMotor = hardwareMap.get(DcMotorEx.class, "motor_drive_flAsDcMotor");
        motor_drive_frAsDcMotor = hardwareMap.get(DcMotorEx.class, "motor_drive_frAsDcMotor");
        motor_drive_blAsDcMotor = hardwareMap.get(DcMotorEx.class, "motor_drive_blAsDcMotor");
        motor_drive_brAsDcMotor = hardwareMap.get(DcMotorEx.class, "motor_drive_brAsDcMotor");
        PIDFCoefficients pidOrig = motor_drive_flAsDcMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

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
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, 90, 0, 90));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
//            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(webcamName, robotFromCamera);
        }
        targetVisible = false;
        waitForStart();
        targetsUltimateGoal.activate();

//AngleAdjustment(5);
//AngleAdjustment(-10);
/*
          DistanceSmoothTravel(12, .6, 0, 0.1, true, true, 1400);
sleep(5000);
        DistanceSmoothTravel(12, .6, 0, 0.1, true, false, 1400);
        sleep(5000);
        */


        wallTargetTracking(vuforia, allTrackables, 90, -3, 40.0, 0, 10, 2, 1, 70000, false, 5);
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
        while ((!(40 - CurrentY <= 1 && 40 - CurrentY >= -1 && -3 - CurrentX <= 1 && -3 - CurrentX >= -1))) {
            if (isStopRequested()) {
                break;
            }
            BulkCaching();
            // express the rotation of the robot in degrees.
            // check all the trackable targets to see which one (if any) is visible.
            telemetry.addData("YOffset", 40 - CurrentY);
            telemetry.addData("XOffset", -3 - CurrentX);
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
                    MecanumFunction(1 * (-0.004 * (xTarget - CurrentX)), (1 * (0.018 * (yTarget - CurrentY))), TrueTrackSwitch * (-0.0075 * (yawTarget - (CurrentVal))) + (-1 * IMUTrackSwitch * -0.01 * (-TargetAngle + CurrentHeading)));
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
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        PIDFCoefficients brpidNew = new PIDFCoefficients(35, 25, 0, 0);
        motor_drive_flAsDcMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        motor_drive_frAsDcMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        motor_drive_blAsDcMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        motor_drive_brAsDcMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, brpidNew);
    }

    private void readCurrentHeading() {
    }


/*
Liam:
  The maximum power of each motor in the .setPower method is +/- 1.0.  The way MecanumFunction is implemented below
  is not 'correct' in that the calculated value could be as much as +/-3.0 which is not correct.  Please figure out an algorithm
  that will 'scale' the .setPower value to a maximum of 1.0.  This might mean that you will need to adjust the function input
  values so that the resulting setPower value is a maximum of +/-1.0 but keep the inputs keep the same ratio.
 */


    private void MecanumFunction(double YL, double XL, double XR) {
        flScale = (-YL - (XL - XR));
        blScale = (YL - (XL + XR));
        frScale = (-YL + XL - XR);
        brScale = (YL + XL + XR);
        if ((-YL - (XL - XR)) > 1 || (-YL - (XL - XR)) < -1) {
            flOverload = true;
        } else {
            flOverload = false;
        }
        if ((YL - (XL + XR)) > 1 || (YL - (XL + XR)) < -1) {
            blOverload = true;
        } else {
            blOverload = false;
        }
        if ((-YL + XL - XR) > 1 || (-YL + XL - XR) < -1) {
            frOverload = true;
        } else {
            frOverload = false;
        }
        if ((YL + (XL + XR)) > 1 || (YL + (XL + XR)) < -1) {
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
            motor_drive_flAsDcMotor.setPower((-YL - (XL - XR)));
            motor_drive_blAsDcMotor.setPower((YL - (XL + XR)));
            motor_drive_frAsDcMotor.setPower((-YL + XL - XR));
            motor_drive_brAsDcMotor.setPower((YL + (XL + XR)));
        } else {
            motor_drive_flAsDcMotor.setPower((-mYL - (mXL - mXR)));
            motor_drive_blAsDcMotor.setPower((mYL - (mXL + mXR)));
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
            while (motor_drive_flAsDcMotor.getCurrentPosition() >= -EncoderTicks && opModeIsActive()) {
                BulkCaching();
                avgEnc = (motor_drive_flAsDcMotor.getCurrentPosition() + motor_drive_frAsDcMotor.getCurrentPosition()) / 2;
                telemetry.addData("Loop", 1);
                telemetry.addData("Encoder Ticks Target", EncoderTicks);
                telemetry.addData("mFl input", mFl);
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
                telemetry.addData("Loop", 2);
                telemetry.addData("mFl input", mFl);
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

