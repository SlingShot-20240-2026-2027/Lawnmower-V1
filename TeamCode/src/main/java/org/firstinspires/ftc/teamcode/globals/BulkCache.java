package org.firstinspires.ftc.teamcode.globals;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class BulkCache {
    public final HardwareMap hardwareMap;
    List<LynxModule> allHubs;
    public BulkCache(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        List<LynxModule> allHubs = this.hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        this.allHubs = this.hardwareMap.getAll(LynxModule.class);
    }
    public void clearCache(){
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}