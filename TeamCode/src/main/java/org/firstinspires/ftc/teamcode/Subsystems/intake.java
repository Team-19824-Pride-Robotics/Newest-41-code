package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@Configurable
public class intake {

    private final DcMotorEx intake;
    private final DcMotorEx transfer;
    public static double intakeIn;
    public static double intakeOut;

    public double intakePower = 0;

    public intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

    }

    public void init() {
        intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intakeSetPower() {
        intakePower = intakeIn + intakeOut;
    }

    public void intakeSetIdle(){
        intakePower = 0;
    }
    public double getIntakeIn() {
        return intakeIn;
    }
    public double getIntakeOut() {
        return intakeOut;
    }
    public double getIntakePower() {
        return intakePower;
    }
    public void update() {
        intake.setPower(intakePower);
        transfer.setPower(intakePower);
    }

}

