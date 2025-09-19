package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Timer;

@TeleOp
public class intakeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare servo ok
        // Absolutely yes make ID's match configuration
        CRServo intake1 = hardwareMap.get(CRServo.class, "1");
        CRServo intake2 = hardwareMap.get(CRServo.class, "2");






        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive()) {

           if(gamepad1.a){
            intake1.setPower(1);
            intake2.setPower(-1);
           }
           if(gamepad1.b){
            intake1.setPower(-1);
            intake2.setPower(1);
           }
           if(gamepad1.x){
            intake1.setPower(0);
            intake2.setPower(0);
           }
        }
    }
}