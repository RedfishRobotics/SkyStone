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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto_Blue_Foundation", group="Iterative Opmode")
//@Disabled
public class Auto_Blue extends LinearOpMode {

    // Declare OpMode members.
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final String VUFORIA_KEY =
            "AakWoPb/////AAABmcceLlxAEkAsjLyJlcWPyuJR6Zl0+rR7eMLL6GmKRcBYxWFw69oaJoRTuN2z75sn3bf499iR+oVWvgnt8/PykAXvID6FoNo8YK6OaRCPKL0LSrbrAE2RhajjLTKv7CjQy7/YWIsHvoYwcpqosoWf5pbLNqb6x4t+B6Gjp3fJf3zPK4F38pimo+qXt1ovZnui5eK1AV3fJw9wCbvMC6vQBf+w//lnLJM6ChpR8TbwI6oUGLuIcjo3TvItkAlJf6ZWkRfvywPfCkzgaEePDNYNRP5ugZC6HlNd/lCYYi9AGixoXhHsay+dJ1GUaLQq8mIhes41jn+e7Flw268kfwFAtZsp+eQGKoffP3RCmsu37lnl";

    private ElapsedTime runtime = new ElapsedTime();
    //    private DistanceSensor sensorRangeRight = null;
//    private DistanceSensor sensorRangeLeft = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;
    private DcMotor elevatorMotor = null;
//    private Servo leftSkystoneServo = null;
//    private Servo rightSkystoneServo = null;
    private Servo blockRotationServo = null;
    private DcMotor rightIntake  = null;
    private DcMotor leftIntake = null;
    private Servo leftFoundationServo = null;
    private Servo rightFoundationServo = null;
    private Servo rightSkystoneServo = null;
    private Servo leftSkystoneServo = null;
    private Servo intake_Deployment = null;
    public BNO055IMU imu;
    public OpenGLMatrix lastLocation = null;

    public Orientation angles;
    public Acceleration gravity;
    //    private ColorSensor rightColorSensor = null;
//    private ColorSensor leftColorSensor = null;
    String position;

    public MovingAvg gyroErrorAvg = new MovingAvg(30);
    static final double P_DRIVE_COEFF_1 = 0.01;  // Larger is more responsive, but also less accurate
    static final double P_DRIVE_COEFF_2 = 0.25;  // Intenionally large so robot "wiggles" around the target setpoint while driving

    static final double COUNTS_PER_MOTOR_REV = 753;    // The encoder ticks per revolution for andymark 40 motors
    static final double DRIVE_GEAR_REDUCTION = 0.5;     // The gear reduction on our motors
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // The diameter to get the circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

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

    // Class Members
//    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    // private int newtarget = 0;

    // static final int COUNTS_PER_MOTOR_REV = 753;
    // static final int DRIVE_GEAR_REDUCTION = 1;
    // static final int WHEEL_DIAMETER_INCHES = 4;
    // static final int COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES *3);


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");
        leftFoundationServo = hardwareMap.get(Servo.class, "left_Foundation_Servo");
        rightFoundationServo = hardwareMap.get(Servo.class, "right_Foundation_Servo");
        elevatorMotor = hardwareMap.get(DcMotor.class, "Elevator_Motor");
        leftIntake = hardwareMap.get(DcMotor.class, "left_intake");
        rightIntake = hardwareMap.get(DcMotor.class, "right_intake");
        blockRotationServo = hardwareMap.get(Servo.class, "Stone_Rotation_Servo");
        intake_Deployment = hardwareMap.get(Servo.class, "intake_deployment");
        rightSkystoneServo = hardwareMap.get(Servo.class, "right_Skystone_Servo");
        leftSkystoneServo = hardwareMap.get(Servo.class, "left_Skystone_Servo");
//        sensorRangeRight = hardwareMap.get(DistanceSensor.class, "Range_Right");
//        sensorRangeLeft = hardwareMap.get(DistanceSensor.class, "Range_Left");
//        rightColorSensor = hardwareMap.get(ColorSensor.class, "Right_Color_Sensor");
//        leftColorSensor = hardwareMap.get(ColorSensor.class, "Left_Color_Sensor");
//        leftSkystoneServo = hardwareMap.get(Servo.class, "left_Skystone_Servo");
//        rightSkystoneServo = hardwareMap.get(Servo.class, "right_Skystone_Servo");

        // Define and Initialize Motors
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
        elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);

        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        blockRotationServo.setPosition(0.875);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Wait for the game to start (driver presses PLAY)

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters Parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

//         VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        Parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
//        Parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(Parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
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

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, Parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targetsSkyStone.activate();
        composeTelemetry();
        telemetry.update();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            elevatorMotor.setTargetPosition(500);
            elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevatorMotor.setPower(0.75);
            encoderDriveBackwardStraight(0.5, -10, 5, true, 0, false);
            sleep(500);
            intake_Deployment.setPosition(0.75);
            sleep(1000);
            rightSkystoneServo.setPosition(0.17);
            leftSkystoneServo.setPosition(0.6);
            rightIntake.setPower(-0.15);
            leftIntake.setPower(-0.15);
            sleep(500);
            rightIntake.setPower(0.0);
            leftIntake.setPower(0.0);
            rightSkystoneServo.setPosition(0.6);
            leftSkystoneServo.setPosition(0.17);
            intake_Deployment.setPosition(0.4);
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
            encoderStrafeLeft(0.5, 10, 10, 5);
            sleep(500);
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
            sleep(600);
            encoderDriveBackwardStraight(0.5, -12, 5, true, 0, false);
            sleep(500);
            leftFoundationServo.setPosition(0.15);
            rightFoundationServo.setPosition(0.86);
            sleep(1500);
            elevatorMotor.setTargetPosition(-500);
            elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevatorMotor.setPower(0.75);
//            gyroTurn(0.85, -90, 0.025);
            encoderDriveForwardStraight(0.5, 30, 5, true, 90, false);
//            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(600);
            encoderDriveBackwardStraight(0.5, -19, 3, true, 90, false);
            sleep(1000);
            leftFoundationServo.setPosition(0.535);
            rightFoundationServo.setPosition(0.45);
            sleep(1000);
            gyroTurn(0.5, 115, 0.025);
            sleep(500);
            encoderDriveForwardStraight(0.85, 25.5, 5, true, 115, false);
            sleep(500);
            gyroTurn(0.5, 90, 0.025);
            stop();

        }
    }

    public void gyroTurn(double speed, double angle, double coefficient) {

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
    boolean onHeading(double speed, double angle, double PCoeff) {
        int HEADING_THRESHOLD = 5;
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
    public double getError(double targetAngle) {

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
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


    /**
     * Record the current heading and use that as the 0 heading point for gyro reads
     *
     * @return
     */
    void zeroGyro() {
        double headingBias;
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        headingBias = angles.firstAngle;
    }

    void composeTelemetry() {

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

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    // /**
    //  * Read the current heading direction.  Use a heading bias if we recorded one at start to account for drift during
    //  * the init phase of match
    //  *
    //  * @return      Current heading (Z axis)
    //  */
    double readGyro() {
        double headingBias = angles.firstAngle;
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return angles.firstAngle - headingBias;
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

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
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
}
