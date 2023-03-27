package frc.robot.util.sim;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.robot.Robot;
import frc.robot.util.TimingTracer;

public class SimGyroSensorModel{

    SimDeviceSim m_gyroSim;
    SimDouble m_rateSimDouble;
    SimDouble m_yawSimDouble;
    double m_gyroPosReading_deg;

    public SimGyroSensorModel(){
        if (Robot.isSimulation()){
            m_gyroSim = new SimDeviceSim("navX-Sensor[0]");
            m_rateSimDouble = m_gyroSim.getDouble("Rate");
            m_yawSimDouble = m_gyroSim.getDouble("Yaw");
        }
        
    }

    public void resetToPose(Pose2d resetPose){
        if (Robot.isSimulation()){
            m_yawSimDouble.set(resetPose.getRotation().getDegrees() * -1.0);
        }
        
    }

    public void update(Pose2d curRobotPose, Pose2d prevRobotPose){
        if (Robot.isSimulation()){
            double delta = curRobotPose.getRotation().minus(prevRobotPose.getRotation()).getDegrees();
            double gyroRate = (delta)/TimingTracer.getLoopTime(); //Gyro reads backward from sim reference frames.
            // Pass our model of what the sensor would be measuring back into the simGyro object
            // for the embedded code to interact with.
            m_rateSimDouble.set(gyroRate);
            m_yawSimDouble.set(m_yawSimDouble.get() + delta);
        }

        
    }

    public Rotation2d getRotation2d() {
        if(Robot.isSimulation()){
            return Rotation2d.fromDegrees(m_yawSimDouble.get());
        }
        return new Rotation2d();
        
    }
}