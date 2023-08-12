package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.TiltedElevatorSim;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ExtendIO;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.TimingTracer;

import static frc.robot.Constants.ArmConstants.*;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.Function;

public class SimExtendIO extends ExtendIO {

    private final LinearSystem<N2, N1, N1> m_extendPlant =
    LinearSystemId.identifyPositionSystem(
        ARM_EXTEND_KV,
        0.01);
    private final TiltedElevatorSim m_extendSim = new TiltedElevatorSim(
        m_extendPlant,
        DCMotor.getNEO(1),
        EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION,
        EXTEND_DRUM_RADIUS,
        MIN_ARM_LENGTH, MAX_ARM_LENGTH, true);

    public SimExtendIO(Consumer<Runnable> addPeriodic) {
        super(addPeriodic);
        //m_extendSim.setState(VecBuilder.fill(STOW_POSITION.armLength, 0));
        addPeriodic.accept(this::simulationPeriodic);
    }

    private void simulationPeriodic() {
        m_extendSim.setAngleFromHorizontal(m_angleSupplier.getAsDouble());

        m_extendSim.update(TimingTracer.getLoopTime());
    }

    @Override
    public double getLength() {
        return m_extendSim.getPositionMeters();
    }

    @Override
    public void setVolts(double volts) {
        m_extendSim.setInputVoltage(NomadMathUtil.subtractkS(MathUtil.clamp(volts, -12, 12), ARM_EXTEND_KS));
    }

    @Override
    public double getVelocity() {
        return m_extendSim.getVelocityMetersPerSecond();
    }

    @Override
    public boolean isHomed() {
        return m_extendSim.hasHitLowerLimit();
    }

    @Override
    public void onHome() {
        super.onHome();
        m_extendSim.setState(VecBuilder.fill(MIN_ARM_LENGTH, 0));
    }
    
}
