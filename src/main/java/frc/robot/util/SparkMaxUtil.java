package frc.robot.util;

import java.net.http.HttpClient.Redirect;
import java.util.function.Function;
import java.util.function.Supplier;

import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DriverStation;

public class SparkMaxUtil {
    public static void confirm(Supplier<REVLibError> setting) {
        for (int i = 0; i < 5; i++) {
            var error = setting.get();
            if (error == REVLibError.kOk) {
                break;
            }
            else {
                DriverStation.reportError("RevLibError " + error.toString(), true);
            }
        }
        
        
    }
}
