package frc.sim.mechanismSimulations.differitalDrive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class DifferitalDriveConfig {
        public final DifferitalDriveCanIds candIds;
        public final DifferentialDrivePhysicsConfig physicsConfig;

        public DifferitalDriveConfig(DifferitalDriveCanIds candIds, DifferentialDrivePhysicsConfig physicsConfig) {
                this.candIds = candIds;
                this.physicsConfig = physicsConfig;
        }

        public static DifferitalDriveConfig defaultConfig(){
                return new DifferitalDriveConfig(DifferitalDriveCanIds.defaultConfig(), DifferentialDrivePhysicsConfig.defaultConfig());
        }

        public static final record DifferitalDriveCanIds(int rightDriveLead_id,
                        int rightDriveFollow_id,
                        int leftDriveLead_id,
                        int leftDriveFollow_id) {
                public static DifferitalDriveCanIds defaultConfig() {
                        return new DifferitalDriveCanIds(1, 2, 3, 4);
                }
        }

        public static final record DifferentialDrivePhysicsConfig(
                        Mass MASS_KG,
                        double GEARING,
                        Distance WHEEL_RADIUS,
                        Distance TRACK_WIDTH,
                        double ROBOT_MOI,
                        Matrix<N7, N1> DEFUALT_STATE) {
                public static DifferentialDrivePhysicsConfig defaultConfig() {
                        return new DifferentialDrivePhysicsConfig(
                                        Kilograms.of(25),
                                        1,
                                        Inches.of(2.5),
                                        Inches.of(24),
                                        8,
                                        MatBuilder.fill(N7.instance, N1.instance, 8.27, 4.05, 0, 0, 0, 0, 0)); // Center of field, no velocity
                }
        }
}
