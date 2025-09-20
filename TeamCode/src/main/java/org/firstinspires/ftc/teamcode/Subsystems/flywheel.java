package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class flywheel {

    private final DcMotorEx flywheel;
    private final DcMotorEx flywheelB;

    public double flywheelPower = 0;

    public flywheel(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelB = hardwareMap.get(DcMotorEx.class, "flywheelB");

    }

    public void init() {
        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }




    public double getVelocity() {
        return flywheel.getVelocity();
    }

    public void update(double intakePower) {
        flywheelB.setPower(intakePower);
        flywheel.setPower(intakePower);
    }
}
