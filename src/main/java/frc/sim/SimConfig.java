package frc.sim;

import static edu.wpi.first.units.Units.Hertz;

import edu.wpi.first.units.measure.Frequency;

public class SimConfig {
    public Frequency physicsUpdateFreq;
    public Frequency networkTableUpdateFreq;


    public static final SimConfig DEFAULT_CONFIG = new SimConfig(Hertz.of(100),Hertz.of(50));


    public SimConfig(Frequency physicsUpdateFreq, Frequency networkTableUpdateFreq) {
        this.physicsUpdateFreq = physicsUpdateFreq;
        this.networkTableUpdateFreq = networkTableUpdateFreq;
    }
}
