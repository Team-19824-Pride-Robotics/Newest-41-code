package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class launcherTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare motor ok
        // Absolutely yes make ID's match configuration

        DcMotor flyWheel = hardwareMap.dcMotor.get("flyWheel");


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.a) {
                flyWheel.setPower(1);
            }
            if (!gamepad1.a && !gamepad1.b) {
                flyWheel.setPower(0);
            }
            if (gamepad1.b) {
                flyWheel.setPower(-1);

            }
        }
    }
}