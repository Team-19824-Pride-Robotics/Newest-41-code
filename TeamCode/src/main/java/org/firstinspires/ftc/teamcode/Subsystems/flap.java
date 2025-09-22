package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class flap{

    private final Servo flap;
    static double close = 0.5;
    static double open = 0.1;
    boolean state=false;
    public flap(HardwareMap hardwareMap) {
        flap = hardwareMap.get(Servo.class, "flap");
    }

    public void open() {
        state=true;
    }
    public void close() {
        state=false;
    }
    public void update(){
        if(state){
            flap.setPosition(open);
        } else{
            flap.setPosition(close);
        }
    }
}

