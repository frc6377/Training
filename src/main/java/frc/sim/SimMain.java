package frc.sim;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.sim.mechanismSimulations.Mechanism;

public class SimMain {
    public final Time physics_dt;
    public final Time nt_dt;
    List<Mechanism> mechanisms = new ArrayList<>();

    public SimMain(){
        this(SimConfig.DEFAULT_CONFIG);
    }

    public SimMain(SimConfig cfg) {
        physics_dt = Seconds.of(1 / cfg.physicsUpdateFreq.in(Hertz));
        nt_dt = Seconds.of(1 / cfg.networkTableUpdateFreq.in(Hertz));
    }

    public void addMechanism(Mechanism mech){
        mechanisms.add(mech);
    }

    public void bind(TimedRobot robot){
        robot.addPeriodic(this::physicsPeriodic, physics_dt);
    }

    public void physicsPeriodic() {
        for(Mechanism mech : mechanisms){
            mech.update(physics_dt);
        }
    }

    
    public void NTPeriodic() {
        for(Mechanism mech : mechanisms){
            mech.updateNTValues();
        }
    }
}
