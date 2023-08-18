package frc.robot.subsystems.arm;

import java.util.function.Consumer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.VariableLengthArmSim;
import frc.robot.util.TimingTracer;

import static frc.robot.Constants.ArmConstants.*;

public class SimWristIO extends WristIO {

    private final VariableLengthArmSim m_wristSim = new VariableLengthArmSim(
        m_wristPlant,
        DCMotor.getNEO(1),
        1.0 / WRIST_ROTATIONS_PER_MOTOR_ROTATION,
        wristMOI, HAND_LENGTH,
        WRIST_MIN_ANGLE, WRIST_MAX_ANGLE, HAND_MASS_KILOS, true);

    private double m_inputVolts;
    public SimWristIO(Consumer<Runnable> addPeriodic) {
        super(addPeriodic);
        addPeriodic.accept(this::periodic);
        m_wristSim.setState(VecBuilder.fill(STOW_POSITION.wristRadians,0));
        //as the arm raises from 0 to pi/2, the gravity on the wrist goes from -pi/2 to -pi
        m_wristSim.setGravityAngle(-Math.PI/2 - m_pivotAngleSupplier.getAsDouble());
        resetController();
    }

    @Override
    public double getAngle() {
        return m_wristSim.getAngleRads();
    }

    /**
     * runs every code loop in simulation, called in simulation periodic: </p>
     * updates the gravity angle of wrist in simulation, </p>
     * updates the wrist simulation by 0.02 seconds, </p>
     */

    private void periodic() {
        if (DriverStation.isDisabled()) {
            m_wristSim.setInputVoltage(0);
        }
        m_wristSim.setGravityAngle(-Math.PI/2 - m_pivotAngleSupplier.getAsDouble());
        m_wristSim.update(TimingTracer.getLoopTime());
    }

    @Override
    protected void setVoltsInternal(double volts) {
        m_inputVolts = MathUtil.clamp(DriverStation.isEnabled() ? volts : 0, -12, 12);
        m_wristSim.setInputVoltage(m_inputVolts);
        
    }
    @Override
    public double getVolts() {
        return m_inputVolts;
    }
    
}
