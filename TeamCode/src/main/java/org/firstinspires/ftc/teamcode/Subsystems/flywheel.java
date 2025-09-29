package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PIDController;

@Configurable
public class flywheel {

    private final DcMotorEx flywheel;
    private final DcMotorEx flywheelB;

    public double flywheelPower = 0;
    public double flywheelVelocity = 0;
    private static double P = 0;
    private static double I = 0;
    private static double D = 0;
    private static double mI = 0;
    private PIDController pidController = new PIDController(P, I, D, mI);

    public flywheel(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelB = hardwareMap.get(DcMotorEx.class, "flywheelB");

    }

    public void init() {
        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheelB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }




    public double getSpeed() {
        flywheelVelocity=flywheel.getVelocity();
        return flywheelVelocity;
    }

    public void update(double intakePower) {
        flywheelB.setPower(pidController.calculate(intakePower, flywheelB.getVelocity()));
        flywheel.setPower(pidController.calculate(-intakePower, flywheel.getVelocity()));
    }
}
