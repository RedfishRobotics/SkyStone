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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Motor_Test", group="Iterative Opmode")

public class Motor_Test extends LinearOpMode{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftRearDrive = null;
    private DcMotorEx rightRearDrive = null;
//    private DcMotor rightIntake  = null;
//    private DcMotor leftIntake = null;
//    private DcMotor elevatorMotor = null;
//    private CRServo elevatorServo = null;
//    private Servo blockRotationServo = null;
//    private Servo stoneGripServo = null;
//    private Servo leftFoundationServo = null;
//    private Servo rightFoundationServo = null;
//    private Servo SkystoneServo = null;
//    private Servo SkystoneGrip = null;
//    private Servo rightSkystoneServo = null;
//    private Servo leftSkystoneServo = null;
//    private Servo leftSkystoneServo = null;
//    private Servo rightSkystoneServo = null;
//    private Servo intake_Deployment = null;

    private static final double[] Latch = new double[]{0, 1, 2, 3, 4, 5, 6};
    private int currentLatchIndex;
    private boolean upPressedLast;
    private boolean downPressedLast;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode(){

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftRearDrive = hardwareMap.get(DcMotorEx.class, "left_rear_drive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        rightRearDrive = hardwareMap.get(DcMotorEx.class, "right_rear_drive");
//        leftIntake = hardwareMap.get(DcMotor.class, "left_intake");
//        rightIntake = hardwareMap.get(DcMotor.class, "right_intake");
//        elevatorMotor = hardwareMap.get(DcMotor.class, "Elevator_Motor");
        // elevatorServo = hardwareMap.get(CRServo.class, "Elevator_Servo");
//        blockRotationServo = hardwareMap.get(Servo.class, "Stone_Rotation_Servo");
//        stoneGripServo = hardwareMap.get(Servo.class, "Stone_Grip_Servo");
//        SkystoneGrip = hardwareMap.get(Servo.class, "skystone_Grip");
//        leftFoundationServo = hardwareMap.get(Servo.class, "left_Foundation_Servo");
//        rightFoundationServo = hardwareMap.get(Servo.class, "right_Foundation_Servo");
//        SkystoneServo = hardwareMap.get(Servo.class, "skystone_Servo");
//        rightSkystoneServo = hardwareMap.get(Servo.class, "right_Skystone_Servo");
//        leftSkystoneServo =  hardwareMap.get(Servo.class, "left_Skystone_Servo");
//        leftSkystoneServo = hardwareMap.get(Servo.class, "left_Skystone_Servo");
//        rightSkystoneServo = hardwareMap.get(Servo.class, "right_Skystone_Servo");
//        intake_Deployment = hardwareMap.get(Servo.class, "intake_deployment");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
//        leftIntake.setDirection(DcMotor.Direction.REVERSE);
//        rightIntake.setDirection(DcMotor.Direction.FORWARD);
//        elevatorMotor.setDirection(DcMotor.Direction.FORWARD);

//        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftFrontDrive.setVelocityPIDFCoefficients(1.48,0.148, 0, 14.8);
//        leftRearDrive.setVelocityPIDFCoefficients(1.48,0.148, 0, 14.8);
//        rightFrontDrive.setVelocityPIDFCoefficients(1.48,0.148, 0, 14.8);
//        rightRearDrive.setVelocityPIDFCoefficients(1.48,0.148, 0, 14.8);

//        leftFrontDrive.setPositionPIDFCoefficients(5.0);
//        leftRearDrive.setPositionPIDFCoefficients(5.0);
//        rightFrontDrive.setPositionPIDFCoefficients(5.0);
//        rightRearDrive.setPositionPIDFCoefficients(5.0);

//        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.currentLatchIndex = 0;
        this.upPressedLast = false;
        this.downPressedLast = false;
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()){
            // Send calculated power to wheels
//             elevatorMotor.setPower(-gamepad2.left_stick_y);
              if(gamepad1.a){
//                  leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                  leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                  rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                  rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                  sleep(1000);
//                  leftFrontDrive.setTargetPosition(2969);
//                  leftRearDrive.setTargetPosition(2960);
//                  rightFrontDrive.setTargetPosition(2960);
//                  rightRearDrive.setTargetPosition(2960);

                  // Turn On RUN_TO_POSITION
//                  leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                  leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                  rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                  rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                  leftFrontDrive.setPower(0.25);
                  rightFrontDrive.setPower(0.25);
                  rightRearDrive.setPower(0.25);
                  leftRearDrive.setPower(0.25);
                  sleep(30000);
//                  leftFrontDrive.setVelocity(0);
//                  rightFrontDrive.setVelocity(0);
//                  rightRearDrive.setVelocity(0);
//                  leftRearDrive.setVelocity(0);
                  leftFrontDrive.setPower(0);
                  leftRearDrive.setPower(0);
                  rightFrontDrive.setPower(0);
                  rightRearDrive.setPower(0);
              }
              if(gamepad1.b){
                  leftFrontDrive.setTargetPosition(0);
//                  leftRearDrive.setTargetPosition(0);
//                  rightFrontDrive.setTargetPosition(0);
//                  rightRearDrive.setTargetPosition(0);

                  // Turn On RUN_TO_POSITION
                  leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                  leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                  rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                  rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//                  leftRearDrive.setPower(0.25);
//                  rightFrontDrive.setPower(0.25);
//                  rightRearDrive.setPower(0.25);
//                  leftRearDrive.setPower(0.25);
//                  leftFrontDrive.setTargetPosition(364);
//                  leftRearDrive.setTargetPosition(364);
//                  rightFrontDrive.setTargetPosition(364);
//                  rightRearDrive.setTargetPosition(364);
//
//                  // Turn On RUN_TO_POSITION
//                  leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                  leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                  rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                  rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                  leftFrontDrive.setVelocity(350);
////                  rightFrontDrive.setVelocity(350);
////                  rightRearDrive.setVelocity(350);
////                  leftRearDrive.setVelocity(350);
//                  leftFrontDrive.setPower(0.25);
//                  leftRearDrive.setPower(0.25);
//                  rightFrontDrive.setPower(0.25);
//                  rightRearDrive.setPower(0.25);
//                  leftFrontDrive.setPower(0.25);
//                  leftRearDrive.setPower(0.25);
//                  rightFrontDrive.setPower(0.25);
//                  rightRearDrive.setPower(0.25);
              }
              if(gamepad1.x){
//                  leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                  leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                  rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                  rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                  sleep(1000);
//                  leftFrontDrive.setPower(0.75);
//                  leftRearDrive.setPower(0.75);
//                  rightFrontDrive.setPower(0.75);
//                  rightRearDrive.setPower(0.75);
                  sleep(30000);
//                  leftFrontDrive.setPower(0);
//                  leftRearDrive.setPower(0);
//                  rightFrontDrive.setPower(0);
//                  rightRearDrive.setPower(0);
              }
            telemetry.addLine("Right Front: " + rightFrontDrive.getCurrentPosition());
            telemetry.addLine("Left Front: " + leftFrontDrive.getCurrentPosition());
            telemetry.addLine("Right Rear: " + rightRearDrive.getCurrentPosition());
            telemetry.addLine("Left Rear: " + leftRearDrive.getCurrentPosition());
            telemetry.update();
//            final boolean upPressed = this.gamepad2.dpad_right;
//            if (upPressed && !this.upPressedLast)
//                this.currentLatchIndex = (this.currentLatchIndex + 1) % Latch.length;
//            this.upPressedLast = upPressed;
//
//            final boolean downPressed = this.gamepad2.dpad_left;
//            if (downPressed && !this.downPressedLast)
//                this.currentLatchIndex = (this.currentLatchIndex - 1) % Latch.length;
//            this.downPressedLast = downPressed;
//
//            if(currentLatchIndex == 0){
//                elevatorMotor.setTargetPosition(400);
//                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                elevatorMotor.setPower(0.75);
//            }
//            if(currentLatchIndex == 1){
//                elevatorMotor.setTargetPosition(1000);
//                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                elevatorMotor.setPower(0.75);
//            }
//            if(currentLatchIndex == 2){
//                elevatorMotor.setTargetPosition(2450);
//                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                elevatorMotor.setPower(0.75);
//            }
//            if(currentLatchIndex == 3){
//                elevatorMotor.setTargetPosition(3850);
//                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                elevatorMotor.setPower(0.75);
//            }
//            if(currentLatchIndex == 4){
//                elevatorMotor.setTargetPosition(5250);
//                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                elevatorMotor.setPower(0.75);
//            }
//            if(currentLatchIndex == 5){
//                elevatorMotor.setTargetPosition(6750);
//                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                elevatorMotor.setPower(0.75);
//            }
//            if(currentLatchIndex == 6){
//                elevatorMotor.setTargetPosition(8000);
//                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                elevatorMotor.setPower(0.75);
//            }
////            if(gamepad2.dpad_down){
////                elevatorMotor.setTargetPosition(1000);
////                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                elevatorMotor.setPower(0.75);
////            }
////            if(gamepad2.dpad_up){
////                elevatorMotor.setTargetPosition(10);
////                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                elevatorMotor.setPower(0.75);
////            }
////            if(gamepad2.dpad_right){
////                elevatorMotor.setTargetPosition(1800);
////                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                elevatorMotor.setPower(0.75);
////            }
////            if(gamepad1.x){
////                intake_Deployment.setPosition(0.75);
////                sleep(1000);
////                rightIntake.setPower(-0.15);
////                leftIntake.setPower(-0.15);
////                sleep(500);
////                rightIntake.setPower(0.0);
////                leftIntake.setPower(0.0);
////                intake_Deployment.setPosition(0.4);
////            }
////            if(gamepad1.y){
////                intake_Deployment.setPosition(0.40);
////            }
////            if(gamepad1.a){
////                intake_Deployment.setPosition(0.75);
////            }
//            if(gamepad2.left_bumper){
//                blockRotationServo.setPosition(0.15);
//            }
//            if(gamepad2.right_bumper){
//                blockRotationServo.setPosition(0.885);
//            }
////            if(gamepad1.a){
////                elevatorMotor.setTargetPosition(250);
////                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                elevatorMotor.setPower(0.75);
////            }
////            if(gamepad1.b){
////                elevatorMotor.setTargetPosition(10);
////                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                elevatorMotor.setPower(0.75);
////            }
////            if(gamepad1.x){
////                elevatorMotor.setTargetPosition(1100);
////                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                elevatorMotor.setPower(0.75);
////            }
//            if(gamepad1.x){//up
////                SkystoneServo.setPosition(0.735);
//                rightSkystoneServo.setPosition(0.6);
//                leftSkystoneServo.setPosition(0.17);
//            }
//            if (gamepad1.b){//down
////                SkystoneServo.setPosition(0.3);
//                rightSkystoneServo.setPosition(0.17);
//                leftSkystoneServo.setPosition(0.6);
//            }
//            if(gamepad1.y){
//                SkystoneGrip.setPosition(0.95);
//            }
//            if(gamepad1.a){//closed
//                SkystoneGrip.setPosition(0.5);
//            }
//            if(gamepad2.dpad_down){
//                elevatorMotor.setTargetPosition(0);
//                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                elevatorMotor.setPower(1.0);
//                sleep(1000);
//                stoneGripServo.setPosition(0.375);
//                sleep(500);
//            }
//            if(gamepad1.right_bumper){
//                elevatorMotor.setTargetPosition(0);
//                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                elevatorMotor.setPower(1.0);
//                sleep(1000);
//                stoneGripServo.setPosition(0.40);
//                sleep(500);
//            }
//            if(gamepad1.right_bumper){
//                elevatorMotor.setTargetPosition(0);
//                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                elevatorMotor.setPower(1.0);
//                sleep(1000);
//                stoneGripServo.setPosition(0.40);
//                sleep(500);
//            }
//            if(gamepad1.left_bumper){
//                stoneGripServo.setPosition(0.0);
//            }
////            if(gamepad1.right_bumper){
////                blockRotationServo.setPosition(0.5);
////            }
////            if(gamepad2.x){
////                SkystoneServo.setPosition(0.95);
////            }
////            if(gamepad2.b){
////                SkystoneServo.setPosition(0.35);
////            }
//            if(gamepad1.left_stick_x < -0.2 && gamepad1.right_stick_x < -0.2){
//                leftFrontDrive.setPower(-0.55);
//                leftRearDrive.setPower(0.5);
//                rightFrontDrive.setPower(0.55);
//                rightRearDrive.setPower(-0.5);
//            }else if(gamepad1.left_stick_x > 0.2 && gamepad1.right_stick_x > 0.2){
//                leftFrontDrive.setPower(0.55);
//                leftRearDrive.setPower(-0.5);
//                rightFrontDrive.setPower(-0.55);
//                rightRearDrive.setPower(0.5);
//            }else if(gamepad1.left_trigger > 0.2) {
//                leftFrontDrive.setPower(0.55);
//                leftRearDrive.setPower(0.55);
//                rightFrontDrive.setPower(0.30);
//                rightRearDrive.setPower(0.30);
//            }else if(gamepad1.right_trigger > 0.2) {
//                leftFrontDrive.setPower(0.30);
//                leftRearDrive.setPower(0.30);
//                rightFrontDrive.setPower(0.55);
//                rightRearDrive.setPower(0.55);
//            }
//            else{
//                leftFrontDrive.setPower(-gamepad1.left_stick_y * .65);
//                leftRearDrive.setPower(-gamepad1.left_stick_y * .65);
//                rightFrontDrive.setPower(-gamepad1.right_stick_y * .65);
//                rightRearDrive.setPower(-gamepad1.right_stick_y *.65);
//            }
//            if(gamepad2.a){
//                rightIntake.setPower(0.35);
//                leftIntake.setPower(0.35);
//            }
//            if(gamepad2.b){
//                rightIntake.setPower(-0.25);
//                leftIntake.setPower(-0.25);
//            }
//            if(gamepad2.x){
//                rightIntake.setPower(0.0);
//                leftIntake.setPower(0.0);
//            }
//            if(gamepad1.dpad_down){
//                leftFoundationServo.setPosition(0.535);
//                rightFoundationServo.setPosition(0.45);
//            }
//            if(gamepad1.dpad_up){
//                leftFoundationServo.setPosition(0.15);
//                rightFoundationServo.setPosition(0.85);
//            }
            // Show the elapsed game time and wheel power.
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Elevator Position: ", currentLatchIndex);
//            telemetry.addData("Elevator Encoder: ", elevatorMotor.getCurrentPosition());
//            telemetry.addLine("Position: " + elevatorMotor.getCurrentPosition());
//            telemetry.update();
        }
        /*
         * Code to run ONCE after the driver hits STOP
         */
    }
}

