package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.teamcode.Subsystems.intake;
import org.firstinspires.ftc.teamcode.Subsystems.flywheel;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class motorDirectionDebugger extends LinearOpMode {
    private DcMotor zero;
    private DcMotor one;
    private DcMotor two;
    private DcMotor three;
    private DcMotor four;
    private DcMotor five;
    private DcMotor six;
    private DcMotor seven;


    private double angleFromGoal;
    private double distanceFromGoal;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare motor ok
        // Absolutely yes make ID's match configuration
        DcMotor zero = hardwareMap.dcMotor.get("0");
        DcMotor one = hardwareMap.dcMotor.get("1");
        DcMotor two = hardwareMap.dcMotor.get("2");
        DcMotor three = hardwareMap.dcMotor.get("3");
        DcMotor four = hardwareMap.dcMotor.get("4");
        DcMotor five = hardwareMap.dcMotor.get("5");
        DcMotor six = hardwareMap.dcMotor.get("6");
        DcMotor seven = hardwareMap.dcMotor.get("7");


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                zero.setPower(1);
            }
            if (gamepad1.b) {
                one.setPower(1);
            }
            if (gamepad1.x) {
                two.setPower(1);
            }
            if (gamepad1.y) {
                three.setPower(1);
            }
            if (gamepad1.dpad_up) {
                four.setPower(1);
            }
            if (gamepad1.dpad_down) {
                five.setPower(1);
            }
            if (gamepad1.dpad_left) {
                six.setPower(1);
            }
            if (gamepad1.dpad_right) {
                seven.setPower(1);
            }
        }
    }
}