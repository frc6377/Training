package frc.sim.mechanismSimulations;

import edu.wpi.first.units.measure.Time;

public interface Mechanism {
    /**
     * Advance the simulation by dt.
     */
    void update(Time dt);

    /**
     * Update the values posed to NT.
     */
    void updateNTValues();
}
