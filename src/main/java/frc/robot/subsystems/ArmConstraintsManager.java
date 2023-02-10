package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class ArmConstraintsManager {
    Field2d VISUALIZER;
    Translation2d[] constraintsHalf = 
    {
        new Translation2d(3.659000,0.503000),//0
        new Translation2d(3.659000,0.887000),//2
        new Translation2d(3.519000,1.198000),//3
        new Translation2d(3.213000,1.118000),//4
        new Translation2d(2.905000,1.154000),//5
        new Translation2d(2.717000,1.231000),//6
        new Translation2d(2.530000,1.365000),//7
        new Translation2d(2.359000,1.568000),//8
        new Translation2d(2.072491,1.235801),//9
        new Translation2d(1.636026,1.083639)//10
        // FLIP
    };
    Translation2d[] constraints = new Translation2d[constraintsHalf.length * 2];
    public ArmConstraintsManager(Field2d VISUALIZER){
        this.VISUALIZER = VISUALIZER;
        for (int i = 0; i < constraintsHalf.length; i++) {
            constraints[i] = constraintsHalf[i];
        }
        for (int i = constraintsHalf.length - 1; i >= 0; i--) {
            constraints[(constraints.length - i) - 1] = new Translation2d( Math.PI -constraintsHalf[i].getX(), constraintsHalf[i].getY());
        }
        // Override for the hand motor stack minimum height limit
        constraints[1] = new Translation2d(3.438, 0.808);
        constraints[2] = new Translation2d(3.361, 1.155);
        setTranslationList(constraints, "constraints");
    }

    private void setTranslationList(Translation2d[] translations, String name) {
        List<Pose2d> poseList = new ArrayList<Pose2d>();
        for (int i = 0; i < translations.length; i++) {
            poseList.add(new Pose2d(translations[i], new Rotation2d()));
        }
        VISUALIZER.getObject(name).setPoses(poseList);
    }
}
