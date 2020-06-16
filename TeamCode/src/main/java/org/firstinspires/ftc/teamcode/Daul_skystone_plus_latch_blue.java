package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *YES
 */
@Autonomous(name= "daul_skystone_plus_latch_Blue", group="Sky autonomous")
//@Disabled//comment out this line before using
public class Daul_skystone_plus_latch_blue extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f / 8f;
    private static float rectWidth = 1.5f / 8f;

    private static float offsetX = 0f / 8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f / 8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f / 8f + offsetX, 4f / 8f + offsetY};//0 = col, 1 = row
    private static float[] leftPos = {1f / 8f + offsetX, 4f / 8f + offsetY};
    private static float[] rightPos = {6.5f / 8f + offsetX, 4f / 8f + offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor/

    private final int rows = 640;
    private final int cols = 480;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AakWoPb/////AAABmcceLlxAEkAsjLyJlcWPyuJR6Zl0+rR7eMLL6GmKRcBYxWFw69oaJoRTuN2z75sn3bf499iR+oVWvgnt8/PykAXvID6FoNo8YK6OaRCPKL0LSrbrAE2RhajjLTKv7CjQy7/YWIsHvoYwcpqosoWf5pbLNqb6x4t+B6Gjp3fJf3zPK4F38pimo+qXt1ovZnui5eK1AV3fJw9wCbvMC6vQBf+w//lnLJM6ChpR8TbwI6oUGLuIcjo3TvItkAlJf6ZWkRfvywPfCkzgaEePDNYNRP5ugZC6HlNd/lCYYi9AGixoXhHsay+dJ1GUaLQq8mIhes41jn+e7Flw268kfwFAtZsp+eQGKoffP3RCmsu37lnl";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    public MovingAvg gyroErrorAvg = new MovingAvg(30);
    static final double P_DRIVE_COEFF_1 = 0.025;  // Larger is more responsive, but also less accurate
    static final double P_DRIVE_COEFF_2 = 0.25;  // Intenionally large so robot "wiggles" around the target setpoint while driving

    static final double COUNTS_PER_MOTOR_REV = 753;    // The encoder ticks per revolution for andymark 40 motors
    static final double DRIVE_GEAR_REDUCTION = 0.5;     // The gear reduction on our motors
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // The diameter to get the circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    OpenCvCamera webcam;


    public BNO055IMU imu;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;
    private Servo rightSkystoneServo = null;
    private Servo leftSkystoneServo = null;
    private DcMotor rightIntake  = null;
    private DcMotor leftIntake = null;
    private DcMotor elevatorMotor = null;
    private Servo leftFoundationServo = null;
    private Servo rightFoundationServo = null;
    private Servo SkystoneServo = null;
    private Servo SkystoneGrip = null;
    private Servo intake_Deployment = null;
    public OpenGLMatrix lastLocation = null;

    public Orientation angles;
    public Acceleration gravity;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");
        leftIntake = hardwareMap.get(DcMotor.class, "left_intake");
        rightIntake = hardwareMap.get(DcMotor.class, "right_intake");
        elevatorMotor = hardwareMap.get(DcMotor.class, "Elevator_Motor");
        intake_Deployment = hardwareMap.get(Servo.class, "intake_deployment");
        SkystoneGrip = hardwareMap.get(Servo.class, "skystone_Grip");
        SkystoneServo = hardwareMap.get(Servo.class, "skystone_Servo");
        rightSkystoneServo = hardwareMap.get(Servo.class, "right_Skystone_Servo");
        leftSkystoneServo = hardwareMap.get(Servo.class, "left_Skystone_Servo");
        leftFoundationServo = hardwareMap.get(Servo.class, "left_Foundation_Servo");
        rightFoundationServo = hardwareMap.get(Servo.class, "right_Foundation_Servo");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        // set references for config
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "IMU");
        imu.initialize(parameters);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);
        elevatorMotor.setDirection(DcMotor.Direction.FORWARD);

        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);//remove this

        webcam.openCameraDevice();//open camera
        webcam.setPipeline(new StageSwitchingPipeline());//different stages
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

//        composeTelemetry();
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            if (valRight == 0) {
                encoderStrafeRight(0.5, 21.5, 21.5, 8);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(500);
                encoderDriveForwardStraight(0.5, 22, 20,true, 0, false);
                leftSkystoneServo.setPosition(0.6);
                sleep(500);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(500);
                encoderStrafeLeft(0.5, 6, 6, 3);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                encoderDriveBackwardStraight(0.75, -60, 36, true, 0, false);
                leftSkystoneServo.setPosition(0.17);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(750);
                encoderDriveForwardStraight(0.65, 40, 20, true, 0, false);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(500);
                encoderStrafeRight(0.5, 6, 6, 3);
                leftSkystoneServo.setPosition(0.6);
                sleep(500);
                encoderStrafeLeft(0.5, 6, 6, 3);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(500);
                encoderDriveBackwardStraight(1.0, -36, 20, true, 0, false);
                leftSkystoneServo.setPosition(0.17);
                sleep(500);
                encoderDriveBackwardStraight(1.0, -25, 20, true, 0 , false);
                sleep(500);
                gyroTurn(0.85, -90, 0.025);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(500);
                encoderDriveBackwardStraight(0.65, -9, 20, true, -90, false);
                leftFoundationServo.setPosition(0.15);
                rightFoundationServo.setPosition(0.86);
                sleep(1000);
                encoderDriveForwardStraight(0.85, 26, 20, true, -90, false);
                leftFoundationServo.setPosition(0.535);
                rightFoundationServo.setPosition(0.45);
                sleep(500);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(500);
                encoderStrafeRight(0.5, 40, 40, 10);
                telemetry.addLine("Left");
                telemetry.update();
                stop();
            }else if(valMid == 0) {
                encoderStrafeRight(0.5, 21.5, 21.5, 8);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(500);
                encoderDriveForwardStraight(0.5, 13, 20,true, 0, false);
                leftSkystoneServo.setPosition(0.6);
                sleep(500);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(500);
                encoderStrafeLeft(0.5, 6, 6, 3);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                encoderDriveBackwardStraight(0.75, -53, 36, true, 0, false);
                leftSkystoneServo.setPosition(0.17);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(750);
                encoderDriveForwardStraight(0.65, 33, 20, true, 0, false);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(500);
                encoderStrafeRight(0.5, 6, 6, 3);
                leftSkystoneServo.setPosition(0.6);
                sleep(500);
                encoderStrafeLeft(0.5, 6, 6, 3);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(500);
                encoderDriveBackwardStraight(1.0, -33, 20, true, 0, false);
                leftSkystoneServo.setPosition(0.17);
                sleep(500);
                encoderDriveBackwardStraight(1.0, -23, 20, true, 0 , false);
                sleep(500);
                gyroTurn(0.85, -90, 0.025);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(500);
                encoderDriveBackwardStraight(0.65, -9, 20, true, -90, false);
                leftFoundationServo.setPosition(0.15);
                rightFoundationServo.setPosition(0.86);
                sleep(1000);
                encoderDriveForwardStraight(0.85, 26, 20, true, -90, false);
                leftFoundationServo.setPosition(0.535);
                rightFoundationServo.setPosition(0.45);
                sleep(500);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(500);
                encoderStrafeRight(0.5, 40, 40, 10);
                telemetry.addLine("Center");
                telemetry.update();
                stop();
            }else if(valLeft == 0){
                encoderStrafeRight(0.5, 21.5, 21.5, 8);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(500);
                encoderDriveForwardStraight(0.5, 8, 20,true, 0, false);
                leftSkystoneServo.setPosition(0.6);
                sleep(500);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(500);
                encoderStrafeLeft(0.5, 6, 6, 3);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                encoderDriveBackwardStraight(0.75, -46, 36, true, 0, false);
                leftSkystoneServo.setPosition(0.17);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(750);
                encoderDriveForwardStraight(0.65, 25, 20, true, 0, false);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(500);
                encoderStrafeRight(0.5, 6, 6, 3);
                leftSkystoneServo.setPosition(0.6);
                sleep(500);
                encoderStrafeLeft(0.5, 6, 6, 3);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(500);
                encoderDriveBackwardStraight(1.0, -25, 20, true, 0, false);
                leftSkystoneServo.setPosition(0.17);
                sleep(500);
                encoderDriveBackwardStraight(1.0, -22, 20, true, 0 , false);
                sleep(500);
                gyroTurn(0.85, -90, 0.025);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(500);
                encoderDriveBackwardStraight(0.65, -9, 20, true,  -90, false);
                leftFoundationServo.setPosition(0.15);
                rightFoundationServo.setPosition(0.86);
                sleep(1000);
                encoderDriveForwardStraight(0.85, 26, 20, true, -90, false);
                leftFoundationServo.setPosition(0.535);
                rightFoundationServo.setPosition(0.45);
                sleep(500);
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(500);
                encoderStrafeRight(0.5, 40, 40, 10);
                telemetry.addLine("Right");
                telemetry.update();
                stop();
            }
            telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();
            sleep(100);
            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);
//             stop();
        }
    }

        //detection pipeline
        static class StageSwitchingPipeline extends OpenCvPipeline {
            Mat yCbCrChan2Mat = new Mat();
            Mat thresholdMat = new Mat();
            Mat all = new Mat();
            List<MatOfPoint> contoursList = new ArrayList<>();

            enum Stage {//color difference. greyscale
                detection,//includes outlines
                THRESHOLD,//b&w
                RAW_IMAGE,//displays raw view
            }

            private Stage stageToRenderToViewport = Stage.detection;
            private Stage[] stages = Stage.values();

            @Override
            public void onViewportTapped() {
                /*
                 * Note that this method is invoked from the UI thread
                 * so whatever we do here, we must do quickly.
                 */

                int currentStageNum = stageToRenderToViewport.ordinal();

                int nextStageNum = currentStageNum + 1;

                if (nextStageNum >= stages.length) {
                    nextStageNum = 0;
                }

                stageToRenderToViewport = stages[nextStageNum];
            }

            @Override
            public Mat processFrame(Mat input) {
                contoursList.clear();
                /*
                 * This pipeline finds the contours of yellow blobs such as the Gold Mineral
                 * from the Rover Ruckus game.
                 */

                //color diff cb.
                //lower cb = more blue = skystone = white
                //higher cb = less blue = yellow stone = grey
                Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
                Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

                //b&w
                Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

                //outline/contour
                Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
                yCbCrChan2Mat.copyTo(all);//copies mat object
                //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


                //get values from frame
                double[] pixMid = thresholdMat.get((int) (input.rows() * midPos[1]), (int) (input.cols() * midPos[0]));//gets value at circle
                valMid = (int) pixMid[0];

                double[] pixLeft = thresholdMat.get((int) (input.rows() * leftPos[1]), (int) (input.cols() * leftPos[0]));//gets value at circle
                valLeft = (int) pixLeft[0];

                double[] pixRight = thresholdMat.get((int) (input.rows() * rightPos[1]), (int) (input.cols() * rightPos[0]));//gets value at circle
                valRight = (int) pixRight[0];

                //create three points
                Point pointMid = new Point((int) (input.cols() * midPos[0]), (int) (input.rows() * midPos[1]));
                Point pointLeft = new Point((int) (input.cols() * leftPos[0]), (int) (input.rows() * leftPos[1]));
                Point pointRight = new Point((int) (input.cols() * rightPos[0]), (int) (input.rows() * rightPos[1]));

                //draw circles on those points
                Imgproc.circle(all, pointMid, 5, new Scalar(255, 0, 0), 1);//draws circle
                Imgproc.circle(all, pointLeft, 5, new Scalar(255, 0, 0), 1);//draws circle
                Imgproc.circle(all, pointRight, 5, new Scalar(255, 0, 0), 1);//draws circle

                //draw 3 rectangles
                Imgproc.rectangle(//1-3
                        all,
                        new Point(
                                input.cols() * (leftPos[0] - rectWidth / 2),
                                input.rows() * (leftPos[1] - rectHeight / 2)),
                        new Point(
                                input.cols() * (leftPos[0] + rectWidth / 2),
                                input.rows() * (leftPos[1] + rectHeight / 2)),
                        new Scalar(0, 255, 0), 3);
                Imgproc.rectangle(//3-5
                        all,
                        new Point(
                                input.cols() * (midPos[0] - rectWidth / 2),
                                input.rows() * (midPos[1] - rectHeight / 2)),
                        new Point(
                                input.cols() * (midPos[0] + rectWidth / 2),
                                input.rows() * (midPos[1] + rectHeight / 2)),
                        new Scalar(0, 255, 0), 3);
                Imgproc.rectangle(//5-7
                        all,
                        new Point(
                                input.cols() * (rightPos[0] - rectWidth / 2),
                                input.rows() * (rightPos[1] - rectHeight / 2)),
                        new Point(
                                input.cols() * (rightPos[0] + rectWidth / 2),
                                input.rows() * (rightPos[1] + rectHeight / 2)),
                        new Scalar(0, 255, 0), 3);

                switch (stageToRenderToViewport) {
                    case THRESHOLD: {
                        return thresholdMat;
                    }

                    case detection: {
                        return all;
                    }

                    case RAW_IMAGE: {
                        return input;
                    }

                    default: {
                        return input;
                    }
                }
            }

        }
        public void gyroTurn ( double speed, double angle, double coefficient){

            telemetry.addLine("DM10337- gyroTurn start  speed:" + speed +
                    "  heading:" + angle);

            // keep looping while we are still active, and not on heading.
            while (opModeIsActive() && !onHeading(speed, angle, coefficient)) {
                // Allow time for other processes to run.
                // onHeading() does the work of turning us
                sleep(1);
                ;
            }

            telemetry.addLine("DM10337- gyroTurn done   heading actual:" + readGyro());
        }


        /**
         * Perform one cycle of closed loop heading control.
         *
         * @param speed  Desired speed of turn.
         * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
         *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
         *               If a relative angle is required, add/subtract from current heading.
         * @param PCoeff Proportional Gain coefficient
         * @return
         */
        boolean onHeading ( double speed, double angle, double PCoeff){
            int HEADING_THRESHOLD = 2;
            double error;
            double steer;
            boolean onTarget = false;
            double leftSpeed;
            double rightSpeed;

            // determine turn power based on +/- error
            error = getError(angle);

            if (Math.abs(error) <= HEADING_THRESHOLD) {
                // Close enough so no need to move
                steer = 0.0;
                leftSpeed = 0.0;
                rightSpeed = 0.0;
                onTarget = true;
            } else {
                // Calculate motor powers
                steer = getSteer(error, PCoeff);
                rightSpeed = speed * steer;
                leftSpeed = -rightSpeed;
            }

            // Send desired speeds to motors.
            leftFrontDrive.setPower(leftSpeed);
            rightFrontDrive.setPower(rightSpeed);
            leftRearDrive.setPower(leftSpeed);
            rightRearDrive.setPower(rightSpeed);

            return onTarget;
        }

        /**
         * getError determines the error between the target angle and the robot's current heading
         *
         * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
         * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
         * +ve error means the robot should turn LEFT (CCW) to reduce error.
         */
        public double getError ( double targetAngle){

            double robotError;

            // calculate error in -179 to +180 range  (
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            robotError = targetAngle - angles.firstAngle;
            while (robotError > 180) robotError -= 360;
            while (robotError <= -180) robotError += 360;
            return robotError;
        }

        /**
         * returns desired steering force.  +/- 1 range.  +ve = steer left
         *
         * @param error  Error angle in robot relative degrees
         * @param PCoeff Proportional Gain Coefficient
         * @return
         */
        public double getSteer ( double error, double PCoeff){
            return Range.clip(error * PCoeff, -1, 1);
        }


        /**
         * Record the current heading and use that as the 0 heading point for gyro reads
         *
         * @return
         */
        void zeroGyro () {
            double headingBias;
            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            headingBias = angles.firstAngle;
        }

        void composeTelemetry () {

            // At the beginning of each telemetry update, grab a bunch of data
            // from the IMU that we will then display in separate lines.
            telemetry.addAction(new Runnable() {
                @Override
                public void run() {
                    // Acquiring the angles is relatively expensive; we don't want
                    // to do that in each of the three items that need that info, as that's
                    // three times the necessary expense.
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();
                }
            });

            telemetry.addLine()
                    .addData("status", new Func<String>() {
                        @Override
                        public String value() {
                            return imu.getSystemStatus().toShortString();
                        }
                    })
                    .addData("calib", new Func<String>() {
                        @Override
                        public String value() {
                            return imu.getCalibrationStatus().toString();
                        }
                    });

            telemetry.addLine()
                    .addData("heading", new Func<String>() {
                        @Override
                        public String value() {
                            return formatAngle(angles.angleUnit, angles.firstAngle);
                        }
                    })
                    .addData("roll", new Func<String>() {
                        @Override
                        public String value() {
                            return formatAngle(angles.angleUnit, angles.secondAngle);
                        }
                    })
                    .addData("pitch", new Func<String>() {
                        @Override
                        public String value() {
                            return formatAngle(angles.angleUnit, angles.thirdAngle);
                        }
                    });

            telemetry.addLine()
                    .addData("grvty", new Func<String>() {
                        @Override
                        public String value() {
                            return gravity.toString();
                        }
                    })
                    .addData("mag", new Func<String>() {
                        @Override
                        public String value() {
                            return String.format(Locale.getDefault(), "%.3f",
                                    Math.sqrt(gravity.xAccel * gravity.xAccel
                                            + gravity.yAccel * gravity.yAccel
                                            + gravity.zAccel * gravity.zAccel));
                        }
                    });
        }

        //----------------------------------------------------------------------------------------------
        // Formatting
        //----------------------------------------------------------------------------------------------

        String formatAngle (AngleUnit angleUnit,double angle){
            return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
        }

        String formatDegrees ( double degrees){
            return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
        }


        // /**
        //  * Read the current heading direction.  Use a heading bias if we recorded one at start to account for drift during
        //  * the init phase of match
        //  *
        //  * @return      Current heading (Z axis)
        //  */
        double readGyro () {
            double headingBias = angles.firstAngle;
            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            return angles.firstAngle - headingBias;
        }

        public void encoderDrive ( double speed,
        double leftInches, double rightInches,
        double timeoutS){
            int newLeftTarget;//init the variable newLeftTarget
            int newRightTarget;//init the variable newRightTarget

            // Ensure that the OpMode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftTarget = leftFrontDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                newRightTarget = rightFrontDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
                leftFrontDrive.setTargetPosition(newLeftTarget);
                leftRearDrive.setTargetPosition(newLeftTarget);
                rightFrontDrive.setTargetPosition(newRightTarget);
                rightRearDrive.setTargetPosition(newRightTarget);

                // Turn On RUN_TO_POSITION
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                leftFrontDrive.setPower(speed);
                leftRearDrive.setPower(speed);
                rightFrontDrive.setPower(speed);
                rightRearDrive.setPower(speed);

                //While the motors and OpMode is running, return telemetry on the motors
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            leftFrontDrive.getCurrentPosition(),
                            rightFrontDrive.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                leftFrontDrive.setPower(0);
                leftRearDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightRearDrive.setPower(0);

                // Turn off RUN_TO_POSITION
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
        }

        public void encoderStrafeLeft ( double speed,
        double leftInches, double rightInches,
        double timeoutS){
            int newLeftTarget;//init the variable newLeftTarget
            int newRightTarget;//init the variable newRightTarget

            // Ensure that the OpMode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftTarget = leftFrontDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                newRightTarget = rightFrontDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
                leftFrontDrive.setTargetPosition(newLeftTarget);
                leftRearDrive.setTargetPosition(-newLeftTarget);
                rightFrontDrive.setTargetPosition(-newRightTarget);
                rightRearDrive.setTargetPosition(newRightTarget);

                // Turn On RUN_TO_POSITION
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                leftFrontDrive.setPower(.75);
                leftRearDrive.setPower(.7);
                rightFrontDrive.setPower(.75);
                rightRearDrive.setPower(.7);

                //While the motors and OpMode is running, return telemetry on the motors
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            leftFrontDrive.getCurrentPosition(),
                            rightFrontDrive.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                leftFrontDrive.setPower(0);
                leftRearDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightRearDrive.setPower(0);

                // Turn off RUN_TO_POSITION
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
        }
        public void encoderStrafeRight ( double speed,
        double leftInches, double rightInches,
        double timeoutS){
            int newLeftTarget;//init the variable newLeftTarget
            int newRightTarget;//init the variable newRightTarget

            // Ensure that the OpMode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftTarget = leftFrontDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                newRightTarget = rightFrontDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
                leftFrontDrive.setTargetPosition(-newLeftTarget);
                leftRearDrive.setTargetPosition(newLeftTarget);
                rightFrontDrive.setTargetPosition(newRightTarget);
                rightRearDrive.setTargetPosition(-newRightTarget);

                // Turn On RUN_TO_POSITION
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                leftFrontDrive.setPower(.75);
                leftRearDrive.setPower(.7);
                rightFrontDrive.setPower(.75);
                rightRearDrive.setPower(.7);


                //While the motors and OpMode is running, return telemetry on the motors
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            leftFrontDrive.getCurrentPosition(),
                            rightFrontDrive.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                leftFrontDrive.setPower(0);
                leftRearDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightRearDrive.setPower(0);

                // Turn off RUN_TO_POSITION
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
        }
    public void encoderDriveForwardStraight(double speed,
                                     double distance,
                                     double timeout,
                                     boolean useGyro,
                                     double heading,
                                     boolean aggressive) {

        // Calculated encoder targets
        int newLFTarget;
        int newRFTarget;
        int newLRTarget;
        int newRRTarget;

        // The potentially adjusted current target heading
        double curHeading = heading;

        // Speed ramp on start of move to avoid wheel slip
        final double MINSPEED = 0.30;           // Start at this power
        final double SPEEDINCR = 0.015;         // And increment by this much each cycle
        double curSpeed;                        // Keep track of speed as we ramp

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

//            RobotLog.i("DM10337- Starting encoderDrive speed:" + speed +
//                    "  distance:" + distance + "  timeout:" + timeout +
//                    "  useGyro:" + useGyro + " heading:" + heading + "  maintainRange: " + maintainRange);

            // Calculate "adjusted" distance  for each side to account for requested turn during run
            // Purpose of code is to have PIDs closer to finishing even on curved moves
            // This prevents jerk to one side at stop
            double leftDistance = distance;
            double rightDistance = distance;
            if (useGyro) {
                // We are gyro steering -- are we requesting a turn while driving?
                double headingChange = getError(curHeading) * Math.signum(distance);
                if (Math.abs(headingChange) > 5.0) {
                    //Heading change is significant enough to account for
                    if (headingChange > 0.0) {
                        // Assume 15.25 inch wheelbase
                        // Add extra distance to the wheel on outside of turn
                        rightDistance += Math.signum(distance) * 2 * 3.1415 * 15.25 * headingChange / 360.0;
//                        RobotLog.i("DM10337 -- Turn adjusted R distance:" + rightDistance);
                    } else {
                        // Assume 15.25 inch wheelbase
                        // Add extra distance from the wheel on inside of turn
                        // headingChange is - so this is increasing the left distance
                        leftDistance -= Math.signum(distance) * 2 * 3.1415 * 15.25 * headingChange / 360.0;
//                        RobotLog.i("DM10337 -- Turn adjusted L distance:" + leftDistance);
                    }
                }
            }

            // Determine new target encoder positions, and pass to motor controller
            newLFTarget = leftFrontDrive.getCurrentPosition() + (int) (leftDistance * COUNTS_PER_INCH);
            newLRTarget = leftRearDrive.getCurrentPosition() + (int) (leftDistance * COUNTS_PER_INCH);
            newRFTarget = rightFrontDrive.getCurrentPosition() + (int) (rightDistance * COUNTS_PER_INCH);
            newRRTarget = rightRearDrive.getCurrentPosition() + (int) (rightDistance * COUNTS_PER_INCH);

            while (leftFrontDrive.getTargetPosition() != newLFTarget) {
                leftFrontDrive.setTargetPosition(newLFTarget);
//                sleep(1);
            }
            while (rightFrontDrive.getTargetPosition() != newRFTarget) {
                rightFrontDrive.setTargetPosition(newRFTarget);
//                sleep(1);
            }
            while (leftRearDrive.getTargetPosition() != newLRTarget) {
                leftRearDrive.setTargetPosition(newLRTarget);
//                sleep(1);
            }
            while (rightRearDrive.getTargetPosition() != newRRTarget) {
                rightRearDrive.setTargetPosition(newRRTarget);
//                sleep(1);
            }

            // Turn On motors to RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            speed = Math.abs(speed);    // Make sure its positive
            curSpeed = Math.min(MINSPEED, speed);

            // Set the motors to the starting power
            leftFrontDrive.setPower(Math.abs(curSpeed));
            rightFrontDrive.setPower(Math.abs(curSpeed));
            leftRearDrive.setPower(Math.abs(curSpeed));
            rightRearDrive.setPower(Math.abs(curSpeed));

            // keep looping while we are still active, and there is time left, until at least 1 motor reaches target
            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    leftFrontDrive.getCurrentPosition() <= newLFTarget &&
                    rightFrontDrive.getCurrentPosition() <= newRFTarget &&
                    leftRearDrive.getCurrentPosition() <= newLRTarget &&
                    rightRearDrive.getCurrentPosition() <= newRRTarget) {

                // Ramp up motor powers as needed
                if (curSpeed < speed) {
                    curSpeed += SPEEDINCR;
                }
                double leftSpeed = curSpeed;
                double rightSpeed = curSpeed;

                // Doing gyro heading correction?
                if (useGyro) {

                    // adjust relative speed based on heading
                    double error = getError(curHeading);

                    updateGyroErrorAvg(error);

                    double steer = getSteer(error,
                            (aggressive ? P_DRIVE_COEFF_2 : P_DRIVE_COEFF_1));

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        steer *= -1.0;

                    // Adjust motor powers for heading correction
                    leftSpeed -= steer;
                    rightSpeed += steer;

                    // Normalize speeds if any one exceeds +/- 1.0;
                    double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0) {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                }

                // And rewrite the motor speeds
                leftFrontDrive.setPower(Math.abs(leftSpeed));
                rightFrontDrive.setPower(Math.abs(rightSpeed));
                leftRearDrive.setPower(Math.abs(leftSpeed));
                rightRearDrive.setPower(Math.abs(rightSpeed));

                // Allow time for other processes to run.
                sleep(1);
            }


            // Stop all motion;
            leftFrontDrive.setPower(0);
            leftRearDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightRearDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    public void encoderDriveBackwardStraight(double speed,
                                            double distance,
                                            double timeout,
                                            boolean useGyro,
                                            double heading,
                                            boolean aggressive) {

        // Calculated encoder targets
        int newLFTarget;
        int newRFTarget;
        int newLRTarget;
        int newRRTarget;

        // The potentially adjusted current target heading
        double curHeading = heading;

        // Speed ramp on start of move to avoid wheel slip
        final double MINSPEED = 0.30;           // Start at this power
        final double SPEEDINCR = 0.015;         // And increment by this much each cycle
        double curSpeed;                        // Keep track of speed as we ramp

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

//            RobotLog.i("DM10337- Starting encoderDrive speed:" + speed +
//                    "  distance:" + distance + "  timeout:" + timeout +
//                    "  useGyro:" + useGyro + " heading:" + heading + "  maintainRange: " + maintainRange);

            // Calculate "adjusted" distance  for each side to account for requested turn during run
            // Purpose of code is to have PIDs closer to finishing even on curved moves
            // This prevents jerk to one side at stop
            double leftDistance = distance;
            double rightDistance = distance;
            if (useGyro) {
                // We are gyro steering -- are we requesting a turn while driving?
                double headingChange = getError(curHeading) * Math.signum(distance);
                if (Math.abs(headingChange) > 5.0) {
                    //Heading change is significant enough to account for
                    if (headingChange > 0.0) {
                        // Assume 15.25 inch wheelbase
                        // Add extra distance to the wheel on outside of turn
                        rightDistance += Math.signum(distance) * 2 * 3.1415 * 15.25 * headingChange / 360.0;
//                        RobotLog.i("DM10337 -- Turn adjusted R distance:" + rightDistance);
                    } else {
                        // Assume 15.25 inch wheelbase
                        // Add extra distance from the wheel on inside of turn
                        // headingChange is - so this is increasing the left distance
                        leftDistance -= Math.signum(distance) * 2 * 3.1415 * 15.25 * headingChange / 360.0;
//                        RobotLog.i("DM10337 -- Turn adjusted L distance:" + leftDistance);
                    }
                }
            }

            // Determine new target encoder positions, and pass to motor controller
            newLFTarget = leftFrontDrive.getCurrentPosition() + (int) (leftDistance * COUNTS_PER_INCH);
            newLRTarget = leftRearDrive.getCurrentPosition() + (int) (leftDistance * COUNTS_PER_INCH);
            newRFTarget = rightFrontDrive.getCurrentPosition() + (int) (rightDistance * COUNTS_PER_INCH);
            newRRTarget = rightRearDrive.getCurrentPosition() + (int) (rightDistance * COUNTS_PER_INCH);

            while (leftFrontDrive.getTargetPosition() != newLFTarget) {
                leftFrontDrive.setTargetPosition(newLFTarget);
//                sleep(1);
            }
            while (rightFrontDrive.getTargetPosition() != newRFTarget) {
                rightFrontDrive.setTargetPosition(newRFTarget);
//                sleep(1);
            }
            while (leftRearDrive.getTargetPosition() != newLRTarget) {
                leftRearDrive.setTargetPosition(newLRTarget);
//                sleep(1);
            }
            while (rightRearDrive.getTargetPosition() != newRRTarget) {
                rightRearDrive.setTargetPosition(newRRTarget);
//                sleep(1);
            }

            // Turn On motors to RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            speed = Math.abs(speed);    // Make sure its positive
            curSpeed = Math.min(MINSPEED, speed);

            // Set the motors to the starting power
            leftFrontDrive.setPower(Math.abs(curSpeed));
            rightFrontDrive.setPower(Math.abs(curSpeed));
            leftRearDrive.setPower(Math.abs(curSpeed));
            rightRearDrive.setPower(Math.abs(curSpeed));

            // keep looping while we are still active, and there is time left, until at least 1 motor reaches target
            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    leftFrontDrive.getCurrentPosition() >= newLFTarget &&
                    rightFrontDrive.getCurrentPosition() >= newRFTarget &&
                    leftRearDrive.getCurrentPosition() >= newLRTarget &&
                    rightRearDrive.getCurrentPosition() >= newRRTarget) {

                // Ramp up motor powers as needed
                if (curSpeed < speed) {
                    curSpeed += SPEEDINCR;
                }
                double leftSpeed = curSpeed;
                double rightSpeed = curSpeed;

                // Doing gyro heading correction?
                if (useGyro) {

                    // adjust relative speed based on heading
                    double error = getError(curHeading);

                    updateGyroErrorAvg(error);

                    double steer = getSteer(error,
                            (aggressive ? P_DRIVE_COEFF_2 : P_DRIVE_COEFF_1));

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        steer *= -1.0;

                    // Adjust motor powers for heading correction
                    leftSpeed -= steer;
                    rightSpeed += steer;

                    // Normalize speeds if any one exceeds +/- 1.0;
                    double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0) {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                }

                // And rewrite the motor speeds
                leftFrontDrive.setPower(Math.abs(leftSpeed));
                rightFrontDrive.setPower(Math.abs(rightSpeed));
                leftRearDrive.setPower(Math.abs(leftSpeed));
                rightRearDrive.setPower(Math.abs(rightSpeed));

                // Allow time for other processes to run.
                sleep(1);
            }


            // Stop all motion;
            leftFrontDrive.setPower(0);
            leftRearDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightRearDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    public void updateGyroErrorAvg(double error) {
        gyroErrorAvg.add(Math.abs(error));
    }
    }

