package frc.sim;

import java.util.function.Supplier;

import frc.sim.mechanismSimulations.differitalDrive.DifferitalDriveConfig;
import frc.sim.mechanismSimulations.differitalDrive.DiffiertialDrive;

public class DefaultRobot {
    public static final Supplier<SimMain> HENRY = () -> {
        SimMain main = new SimMain();
        main.addMechanism(new DiffiertialDrive(DifferitalDriveConfig.defaultConfig()));
        return main;
    };
}
