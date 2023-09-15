package frc.robot.subsystems.arm;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.revrobotics.CANSparkMax.IdleMode;

import static frc.robot.Constants.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.VariableLengthArmSim;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.TimingTracer;

public class SimPivotIO extends PivotIO {

    
    private final VariableLengthArmSim m_pivotSim = new VariableLengthArmSim(
        m_pivotPlant,
        DCMotor.getNEO(2),
        1.0/ARM_ROTATIONS_PER_MOTOR_ROTATION,
        1.0 / 3.0 * ARM_MASS_KILOS * MIN_ARM_LENGTH * MIN_ARM_LENGTH,
        MIN_ARM_LENGTH,
        MIN_ARM_ANGLE,
        MAX_ARM_ANGLE,
        ARM_MASS_KILOS,
        true
    );

    private double m_inputVolts;
    
    public SimPivotIO(Consumer<Runnable> addPeriodic, BooleanSupplier hasCone) {
        super(addPeriodic, hasCone);
        addPeriodic.accept(this::simulationPeriodic);
        m_pivotSim.setState(VecBuilder.fill(STOW_POSITION.pivotRadians,0));
        // we need this to calculate outputs
        m_pivotSim.update(0.0001);
        resetController();
        resetGoal();
    }

    @Override
    protected void setVolts(double volts) {
        volts = MathUtil.clamp(DriverStation.isEnabled() ? volts : 0, -12, 12);
        m_inputVolts = NomadMathUtil.subtractkS(volts, PIVOT_KS);
        m_pivotSim.setInputVoltage(m_inputVolts); 
    }

    private void simulationPeriodic() {
        if (DriverStation.isDisabled()) {
            m_pivotSim.setInputVoltage(0);
        }
        m_pivotSim.setCGRadius(getPivotCGRadius());
        m_pivotSim.setMOI(getPivotMOI());
        m_pivotSim.update(TimingTracer.getLoopTime());
    }
    @Override
    public double getContinuousRangeAngle() {
        // TODO Auto-generated method stub
        return continuousRangeAngleModulus(m_pivotSim.getAngleRads());
    }

    public double getVolts() {
        return m_inputVolts;
    }

    @Override
    public void setIdleMode(IdleMode mode) {
        // TODO Auto-generated method stub
        
    }

    @Override
    protected double getVelocity() {
        // TODO Auto-generated method stub
        return m_pivotSim.getVelocityRadPerSec();
    }
    
}
