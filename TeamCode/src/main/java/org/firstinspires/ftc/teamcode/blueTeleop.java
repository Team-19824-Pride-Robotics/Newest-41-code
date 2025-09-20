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
public class blueTeleop extends LinearOpMode {
    private Limelight3A limelight;
    private intake intake;
    private flywheel flyWheel;
    private double currentTime;
    private double currentError;
    private double desiredVelocity;
    private static double P=0;
    private static double I=0;
    private static double D=0;
    private static double mI=0;

    private double angleFromGoal;
    private double distanceFromGoal;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare motor ok
        // Absolutely yes make ID's match configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("bR");

        DcMotor flyWheel = hardwareMap.dcMotor.get("flyWheel");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        intake = new intake(hardwareMap);
        intake.init();

        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead ok.
        // See the note about this earlier on this page ok.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map ok
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot ok
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        //hi twin ok
        imu.initialize(parameters);






        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            // Absolutely yes this button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }


            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation ok
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing ok

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1] ok
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(-backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(-backRightPower);



            //Mechansims

            //intake
            if (gamepad2.left_trigger > .1 || gamepad2.right_trigger > .1) {
                intake.intakeIn = Math.pow(gamepad2.right_trigger, 3);
                intake.intakeOut = Math.pow(-gamepad2.left_trigger, 3);
                intake.intakeSetPower();
            }

            //flywheel
            //currentError = desiredVelocity-flywheel.getVelocity();


            if (gamepad1.a) {
                flyWheel.setPower(1);
            }
            if (!gamepad1.a && !gamepad1.b) {
                flyWheel.setPower(0.00000000000001);
            }
            if (gamepad1.b) {
                flyWheel.setPower(-1);
            }

            // Retrieve the latest result from the limelight
            LLResult result = limelight.getLatestResult();
            if(result != null) {
                angleFromGoal = result.getTx();
                distanceFromGoal = result.getBotposeAvgDist();
            }
            telemetry.addData("Angle from goal: ", angleFromGoal);
            telemetry.addData("Distance From Goal: ", distanceFromGoal);
            telemetry.update();

        }
    }
}