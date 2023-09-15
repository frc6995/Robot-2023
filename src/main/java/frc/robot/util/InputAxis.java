package frc.robot.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import frc.robot.util.drive.AsymmetricSlewRateLimiter;
import autolog.Logged;
import io.github.oblarg.oblog.annotations.Log;

/**
 * An extension of DoubleSupplier designed for pre-processing driver controller axis inputs.
 */
public class InputAxis implements DoubleSupplier{
    DoubleSupplier m_supplier;
    double deadband = 0;
    AsymmetricSlewRateLimiter limiter = new AsymmetricSlewRateLimiter(Double.MAX_VALUE, Double.MAX_VALUE);
    boolean square;
    double multiplier = 1;
    
    double outputValue;
    String name;
    public InputAxis(String name, DoubleSupplier supplier) {
        this.name = name;
        m_supplier = supplier;
    }

    // @Override
    // public String configureLogName() {
    //     return name;
    // }

    private double inputValue() {
        return m_supplier.getAsDouble();
    }

    public InputAxis withSlewRate(double forward, double back) {
        limiter = new AsymmetricSlewRateLimiter(forward, back);
        return this;
    }

    public InputAxis withSlewRate(double rate) {
        limiter = new AsymmetricSlewRateLimiter(rate);
        return this;
    }

    public InputAxis withDeadband(double deadband) {
        this.deadband = deadband;
        return this;
    }

    public InputAxis withSquaring(boolean square) {
        this.square = square;
        return this;
    }

    public InputAxis withInvert(boolean invert) {
        this.multiplier = Math.abs(this.multiplier) * (invert ? -1 : 1);
        return this;
    }

    public void resetSlewRate() {
        double value = m_supplier.getAsDouble();
        value *= multiplier;
        value = MathUtil.applyDeadband(value, deadband);
        if (this.square) {
            value = Math.copySign(value*value, value);
        }
        limiter.reset(value);
    }

    @Override
    public double getAsDouble() {
        double value = m_supplier.getAsDouble();
        value *= multiplier;
        value = MathUtil.applyDeadband(value, deadband);
        if (this.square) {
            value = Math.copySign(value*value, value);
        }
        value = limiter.calculate(value);
        outputValue = value;
        return value;
    }
}
