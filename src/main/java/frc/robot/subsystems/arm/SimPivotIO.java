package frc.robot.subsystems.arm;

import java.util.function.Consumer;

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
        DCMotor.getNEO(1),
        1.0/ARM_ROTATIONS_PER_MOTOR_ROTATION,
        1.0 / 3.0 * ARM_MASS_KILOS * MIN_ARM_LENGTH * MIN_ARM_LENGTH,
        MIN_ARM_LENGTH,
        MIN_ARM_ANGLE,
        MAX_ARM_ANGLE,
        ARM_MASS_KILOS,
        true
    );
    
    public SimPivotIO(Consumer<Runnable> addPeriodic) {
        super(addPeriodic);
        addPeriodic.accept(this::simulationPeriodic);
        m_pivotSim.setState(VecBuilder.fill(STOW_POSITION.pivotRadians,0));
        //TODO Auto-generated constructor stub
    }

    @Override
    protected void setVolts(double volts) {
        volts = MathUtil.clamp(DriverStation.isEnabled() ? volts : 0, -12, 12);
        m_pivotSim.setInputVoltage(NomadMathUtil.subtractkS(volts, PIVOT_KS)); 
    }

    private void simulationPeriodic() {
        m_pivotSim.setCGRadius(getPivotCGRadius());
        m_pivotSim.setMOI(getPivotMOI());
        m_pivotSim.update(TimingTracer.getLoopTime());
    }
    @Override
    public double getContinuousRangeAngle() {
        // TODO Auto-generated method stub
        return continuousRangeAngleModulus(m_pivotSim.getAngleRads());
    }
    
}
