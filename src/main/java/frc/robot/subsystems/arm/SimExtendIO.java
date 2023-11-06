package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
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

import com.revrobotics.CANSparkMax.IdleMode;

public class SimExtendIO extends ExtendIO {


    private final TiltedElevatorSim m_extendSim = new TiltedElevatorSim(
        m_extendPlant,
        DCMotor.getNEO(1),
        EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION,
        EXTEND_DRUM_RADIUS,
        MIN_ARM_LENGTH, MAX_ARM_LENGTH, false);

    private double m_inputVolts;
    public SimExtendIO(Consumer<Runnable> addPeriodic) {
        super(addPeriodic);
        m_extendSim.setState(VecBuilder.fill(STOW_POSITION.armLength, 0));
        m_extendSim.update(0.0001);
        addPeriodic.accept(this::simulationPeriodic);
        resetController();
        resetGoal();
    }

    private void simulationPeriodic() {
        if (DriverStation.isDisabled()) {
            m_inputVolts = 0;
            m_extendSim.setInputVoltage(0);
        }
        m_extendSim.setAngleFromHorizontal(m_angleSupplier.getAsDouble());

        m_extendSim.update(TimingTracer.getLoopTime());
    }

    @Override
    public double getLength() {
        return m_extendSim.getPositionMeters();
    }

    @Override
    public void setVolts(double volts) {

        m_inputVolts = NomadMathUtil.subtractkS(MathUtil.clamp(volts, -12, 12), ARM_EXTEND_KS) - ARM_EXTEND_KG_VERTICAL * Math.sin(m_angleSupplier.getAsDouble());
        if (DriverStation.isDisabled()) {m_inputVolts = 0;}
        m_extendSim.setInputVoltage(m_inputVolts);
    }

    @Override
    public double getVelocity() {
        return m_extendSim.getVelocityMetersPerSecond();
    }

    @Override
    public boolean isHomed() {
        // limit switch button is maybe 5 mm tall
        return getLength() <= MIN_ARM_LENGTH;
    }

    @Override
    public void onHome() {
        super.onHome();
        m_extendSim.setState(VecBuilder.fill(MIN_ARM_LENGTH, 0));
    }
    @Override
    public double getVolts() {
        // TODO Auto-generated method stub
        return m_inputVolts;
    }

    @Override
    public void setIdleMode(IdleMode mode) {
        // TODO Auto-generated method stub
        
    }
    
}
