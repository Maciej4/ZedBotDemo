package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NTInterface {
    public double tx;
    public double ty;
    public double tz;
    public double heartbeat;
    public double pastHeartbeat;
    public boolean goodComms;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("jetson");
    NetworkTableEntry txEntry = table.getEntry("zedTx");
    NetworkTableEntry tyEntry = table.getEntry("zedTy");
    NetworkTableEntry tzEntry = table.getEntry("zedTz");
    NetworkTableEntry heartbeatEntry = table.getEntry("heartbeat");

    public NTInterface ()
    {
        inst.startClientTeam(972);
        inst.startDSClient();
    }

    public void loop(){
        tx = txEntry.getDouble(0.0);
        ty = tyEntry.getDouble(-100.0);
        tz = tzEntry.getDouble(0.0);
        
        if(ty < -99.0) {
            System.out.println("Jetson has not posted values to NetworkTables");
            goodComms = false;
        } else {
            goodComms = true;
        }
    }
}