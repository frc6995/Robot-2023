package frc.robot.util.sim;

/** Add your docs here. */
public class SimEncoder {
    private double mposition = 0;
    private double m_velocity = 0;

    public SimEncoder() {}

    public void setMposition(double position) {
        this.mposition = position;
    }

    public double getMposition() {
        return mposition;
    }

    public void setM_velocity(double velocity) {
        this.m_velocity = velocity;
    }

    public double getM_velocity() {
        return m_velocity;
    }
}
