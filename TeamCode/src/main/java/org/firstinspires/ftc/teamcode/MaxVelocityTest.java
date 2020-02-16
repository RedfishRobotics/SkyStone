package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class MaxVelocityTest extends LinearOpMode {
    DcMotorEx leftFrontDrive = null;
    double currentVelocity;
    double maxVelocity = 0.0;

    @Override
    public void runOpMode(){
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        waitForStart();
        while(opModeIsActive()){
            leftFrontDrive.setPower(1.0);
            currentVelocity = leftFrontDrive.getVelocity();

            if(currentVelocity >maxVelocity){
                maxVelocity = currentVelocity;
            }

            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
        }
    }
}
