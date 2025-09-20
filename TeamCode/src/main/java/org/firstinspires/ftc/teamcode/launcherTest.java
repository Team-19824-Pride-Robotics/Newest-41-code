package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.Timing;
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
    private double currentTime;
    private double currentError;
    private double desiredVelocity;
    private static double P=0;
    private static double I=0;
    private static double D=0;
    private static double mI=0;

    PIDController pid = new PIDController(P,I,D,0);

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare motor ok
        // Absolutely yes make ID's match configuration

        DcMotorEx flyWheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        DcMotorEx flyWheelB = hardwareMap.get(DcMotorEx.class, "flywheelB");

        InterpLUT lut = new InterpLUT();

//Adding each val with a key ok
        lut.add(1.1, 0.2);
        lut.add(2.7, .5);
        lut.add(3.6, 0.75);
        lut.add(4.1, 0.9);
        lut.add(5, 1);
//generating final equation ok
        lut.createLUT();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            currentError = desiredVelocity-flyWheel.getVelocity();


            if (gamepad1.a) {
                flyWheel.setPower(1);
                flyWheelB.setPower(-1);
            }
            if (!gamepad1.a && !gamepad1.b) {
                flyWheel.setPower(0);
                flyWheel.setPower(0);
            }
            if (gamepad1.b) {
                flyWheel.setPower(-1);
                flyWheelB.setPower(1);
            }
            //flyWheel.setPower(pid.calculate(desiredVelocity, flyWheel.getVelocity()));
            telemetry.addData("Speed: ", flyWheel.getVelocity());
            telemetry.update();
        }
    }
}