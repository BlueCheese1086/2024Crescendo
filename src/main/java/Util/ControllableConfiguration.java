package Util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControllableConfiguration {
    
    private final String name;
    private final String subsystem;

    private Object defaultValue;

    private final SendableChooser<?> sendable;

    public ControllableConfiguration(String subsystem, String key, Object defaultValue) {
        this.name = key;
        this.defaultValue = defaultValue;
        this.subsystem = subsystem;
        if (defaultValue instanceof SendableChooser<?>) {
            this.sendable = (SendableChooser<?>) defaultValue;
        } else {
            this.sendable = null;
        }
        setValue(this.defaultValue);
    }

    public void setValue(Object value) {
        String networkKey = String.format("/%s/%s", subsystem, name);
        if (value instanceof Integer || value instanceof Double) {
            SmartDashboard.putNumber(networkKey, value instanceof Integer ? ((Integer)value).doubleValue() : ((Double)value).doubleValue());
        } else if (value instanceof String) {
            SmartDashboard.putString(networkKey, (String) value);
        } else if (value instanceof Boolean) {
            SmartDashboard.putBoolean(networkKey, (Boolean) value);
        } else if (value instanceof SendableChooser<?>) {
            SmartDashboard.putData(networkKey, (SendableChooser<?>) value);
        } else {
            defaultValue = value;
        }
    }

    public Object getValue() {
        String networkKey = String.format("/%s/%s", subsystem, name);
        if (defaultValue instanceof Integer || defaultValue instanceof Double) {
            return (Double) SmartDashboard.getNumber(networkKey, defaultValue instanceof Integer ? ((Integer)defaultValue).doubleValue() : ((Double)defaultValue).doubleValue());
        } else if (defaultValue instanceof String) {
            return (String) SmartDashboard.getString(networkKey, (String) defaultValue);
        } else if (defaultValue instanceof Boolean) {
            return (Boolean) SmartDashboard.getBoolean(networkKey, (Boolean) defaultValue);
        } else if (defaultValue instanceof SendableChooser<?>) {
            return sendable.getSelected();
        }
        return defaultValue;
    }

}