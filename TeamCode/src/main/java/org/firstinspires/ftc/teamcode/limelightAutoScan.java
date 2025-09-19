package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
public class limelightAutoScan extends LinearOpMode {
    private Limelight3A limelight;
    private String pattern;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare stuff ok
        // Absolutely yes make ID's match configuration
       limelight = hardwareMap.get(Limelight3A.class, "limelight");



        // Retrieve the IMU from the hardware map ok
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot ok
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        //hi twin ok
        imu.initialize(parameters);


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
        pattern="N/A";


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.getPipelineIndex()==0) {
                pattern="Green Purple Purple";

            } else if(result != null && result.getPipelineIndex()==1) {
                pattern="Purple Green Purple";
            }
            else if(result != null && result.getPipelineIndex()==2) {
                pattern="Purple Purple Green";
            }
            else{
                pattern="N/A";
            }
            if(gamepad1.a){
                limelight.pipelineSwitch(0);
            }
            if(gamepad1.b){
                limelight.pipelineSwitch(1);
            }
            if(gamepad1.y){
                limelight.pipelineSwitch(2);
            }
            if(gamepad1.right_stick_button){
                telemetry.addData("dat tieu eats cats: ", pattern);
            }
            if(!gamepad1.right_stick_button){
                telemetry.addData("Pattern is: ", pattern);
            }
            telemetry.update();
        }
    }
}