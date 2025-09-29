package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Configurable
public class flap {

    // Dashboard-tunable positions
    public static double closePosition = 0.15;
    public static double openPosition  = 0;

    private final Servo servo;
    private double desiredPosition = closePosition;

    public flap(HardwareMap hardwareMap) {
        this.servo  = hardwareMap.get(Servo.class, "flap");
        servo.setPosition(desiredPosition);
    }

    public void openFlap() {
       servo.setPosition(openPosition);
    }

    public void closeFlap() {
        servo.setPosition(closePosition);
    }

    public void update() {
       
    }
    public double getPosition(){
        return servo.getPosition();
    }
}