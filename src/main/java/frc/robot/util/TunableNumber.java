package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TunableNumber {
    private final String m_key;
    private final double m_defaultValue;
    private final NetworkTableEntry m_entry;

    public TunableNumber(String key, double defaultValue) {
        m_key = "Tunable/" + key;
        m_defaultValue = defaultValue;
        
        // Initialize the value on NetworkTables
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Tuning");
        m_entry = table.getEntry(key);
        m_entry.setDefaultNumber(m_defaultValue);
    }

    public double get() {
        // Retrieve the current value from the dashboard, or default if missing
        return m_entry.getNumber(m_defaultValue).doubleValue();
    }
}