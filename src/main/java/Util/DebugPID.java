package Util;

import java.util.Objects;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DebugPID extends SubsystemBase {

    SparkPIDController controller;
    PIDController controllerGeneric;
    PIDController debug;
    PIDController lastDebug;
    double ff;
    double lastFF;

    ComplexWidget widget;

    ControllableConfiguration kp, ki, kd, kff;

    public DebugPID(SparkPIDController c, String name) {
        this.controller = c;
        debug = new PIDController(
            c.getP(), 
            c.getI(), 
            c.getD()
        );
        this.lastDebug = new PIDController(
            c.getP(), 
            c.getI(), 
            c.getD()
        );
        
        kp = new ControllableConfiguration("Tuning/"+name, "P", controller.getP());
        ki = new ControllableConfiguration("Tuning/"+name, "I", controller.getI());
        kd = new ControllableConfiguration("Tuning/"+name, "D", controller.getD());
        kff = new ControllableConfiguration("Tuning/"+name, "FF", controller.getFF());
            // Shuffleboard.getTab("Debug").add(name, debug);
    }

    public DebugPID(PIDController c, String name) {
        this.controllerGeneric = c;
        debug = new PIDController(
            c.getP(), 
            c.getI(), 
            c.getD()
        );
        this.lastDebug = new PIDController(
            c.getP(), 
            c.getI(), 
            c.getD()
        );
        
        kp = new ControllableConfiguration("Tuning/"+name, "P", controllerGeneric.getP());
        ki = new ControllableConfiguration("Tuning/"+name, "I", controllerGeneric.getI());
        kd = new ControllableConfiguration("Tuning/"+name, "D", controllerGeneric.getD());
        // kff = new ControllableConfiguration("Tuning/"+name, "FF", controller.getFF());
        // Shuffleboard.getTab("Debug").add(name, debug);
    }

    @Override
    public void periodic() {
        debug = new PIDController(
            (Double) kp.getValue(),
            (Double) ki.getValue(),
            (Double) kd.getValue()
        );
        ff = (Double) kff.getValue();
        if (!equals(debug, lastDebug) || ff != lastFF) {
            if (Objects.isNull(controllerGeneric)) {
                controller.setP(debug.getP());
                controller.setI(debug.getI());
                controller.setD(debug.getD());
                controller.setFF(ff);
            } else {
                controllerGeneric.setP(debug.getP());
                controllerGeneric.setI(debug.getI());
                controllerGeneric.setD(debug.getD());
            }
            // controller.setReference(debug.getSetpoint(), ControlType.kPosition);
        }
        lastDebug.setP(debug.getP());
        lastDebug.setI(debug.getI());
        lastDebug.setD(debug.getD());
        lastFF = ff;
    }

    private boolean equals(PIDController one, PIDController two) {
        return 
        one.getP() == two.getP() &&
        one.getI() == two.getI() &&
        one.getD() == two.getD();
    }

    
}