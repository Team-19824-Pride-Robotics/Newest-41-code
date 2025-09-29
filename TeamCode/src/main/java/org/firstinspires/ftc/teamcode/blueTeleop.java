package org.firstinspires.ftc.teamcode;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.teamcode.Subsystems.intake;
import org.firstinspires.ftc.teamcode.Subsystems.flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.flap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Configurable
public class blueTeleop extends LinearOpMode {
    private Limelight3A limelight;
    private intake intake;
    private flywheel flywheel;
    private flap flap;
    private double currentTime;
    private double currentError;
    private double desiredVelocity;
    private static double launchVel=700;
    private double angleFromGoal;
    private double distanceFromGoal;
    private boolean launch=false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare motor ok
        // Absolutely yes make ID's match configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("lF");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("lB");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rF");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rB");


        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        intake = new intake(hardwareMap);
        flap = new flap(hardwareMap);
        flywheel = new flywheel(hardwareMap);
        flywheel.init();
        flap.closeFlap();
        intake.init();

        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(3);
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
            double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad2.left_stick_x;
            double rx = gamepad2.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(-frontLeftPower);
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
            if (gamepad2.left_trigger == 0 || gamepad2.right_trigger == 0) {
                intake.intakeIn = Math.pow(gamepad2.right_trigger, 3);
                intake.intakeOut = Math.pow(-gamepad2.left_trigger, 3);
                intake.intakeSetPower();
            }


            //flywheel
            //currentError = desiredVelocity-flywheel.getVelocity();


            if (gamepad2.a) {
                launch=true;
            }

            if (gamepad2.b) {
                launch=false;
            }

            if (gamepad2.right_bumper) {
                flap.openFlap();
            }
            if (gamepad2.left_bumper) {
                flap.closeFlap();
            }
            if(launch){
                flywheel.update(launchVel);
            } else{
                flywheel.update(0);
            }
             //Retrieve the latest result from the limelight
            LLResult result = limelight.getLatestResult();
            if(result != null) {
                angleFromGoal = result.getTx();
                distanceFromGoal = result.getBotposeAvgDist();
            }
            intake.update();
            flap.update();
            telemetry.addData("Wheel speed ", flywheel.getSpeed());
            telemetry.addData("Distance From Goal: ", distanceFromGoal);
            telemetry.addData("Angle From Goal", angleFromGoal);
            telemetry.update();

        }
    }
}