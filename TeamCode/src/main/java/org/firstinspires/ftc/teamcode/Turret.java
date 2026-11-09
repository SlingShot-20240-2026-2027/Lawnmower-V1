package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "TurretTest", group = "TeleOp")
public class Turret extends OpMode {

    private CRServo turret;


    @Override
    public void init() {

        turret = hardwareMap.get(CRServo.class,"2");


    }

    @Override
    public void loop() {
       turret.setPower(0.8);
    }
}
