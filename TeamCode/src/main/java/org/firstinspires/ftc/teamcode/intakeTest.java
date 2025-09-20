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
    private DcMotor intake;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor intake =hardwareMap.get(DcMotorEx.class, "intake");
        // Declare servo ok
        // Absolutely yes make ID's match configuration







        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive()) {

           if(gamepad1.a){
            intake.setPower(1);
           }
           if(gamepad1.b){
            intake.setPower(-1);
           }
           if(gamepad1.x){
            intake.setPower(0);
           }
        }
    }
}