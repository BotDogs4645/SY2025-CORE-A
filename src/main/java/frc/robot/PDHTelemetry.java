package frc.robot;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class PDHTelemetry {
    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("PDH");
    private final DoublePublisher totalCurrent = table.getDoubleTopic("Total Current").publish();
    private final DoublePublisher voltage = table.getDoubleTopic("Voltage").publish();
    private final DoubleArrayPublisher channelCurrents = table.getDoubleArrayTopic("Current").publish();

    PDHTelemetry() {}
    
    public void telemeterize() {
        totalCurrent.set(pdh.getTotalCurrent());
        voltage.set(pdh.getVoltage());
        double[] currents = new double[pdh.getNumChannels()];
        for (int i=0; i<pdh.getNumChannels(); i++) {
            currents[i] = pdh.getCurrent(i);
        }
        channelCurrents.set(currents);
        
    } 
}