package frc.sim;

import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Robot;

public class SimulatedRobot {
    public static Supplier<TimedRobot> createSimRobot(Supplier<TimedRobot> robotSupplier, Supplier<SimMain> simSupplier){
        return () -> {
            TimedRobot newRobot = robotSupplier.get();
            
            if(Robot.isReal()){
                return newRobot;
            }

            SimMain sim = simSupplier.get();


            // Simulation should be ran at a frequency of 100hz
            newRobot.addPeriodic(sim::physicsPeriodic, sim.physics_dt.in(Seconds));
            newRobot.addPeriodic(sim::NTPeriodic, sim.nt_dt.in(Seconds));

            return newRobot;
        };
    }
}
