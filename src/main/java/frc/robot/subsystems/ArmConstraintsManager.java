package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class ArmConstraintsManager {
    Field2d VISUALIZER;
    static Translation2d[] constraintsHalf = 

    // TODO put points in Constants folder and reference here
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
    
    private static Translation2d[] constraints = new Translation2d[constraintsHalf.length * 2];
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

    /**
    * while traveling from p to q, is it a left turn or right turn to point at r.
    * @return 1 for left, 0 for collinear, -1 for right
    */
    private static int leftRightOn(Translation2d p, Translation2d q, Translation2d r) {
        // q-p = vector from p to q
        // r-p = vecto from p to r
        double areaTwice = (q.getX() - p.getX()) * (r.getY() - p.getY()) 
            - (r.getX() - p.getX()) * (q.getY() - p.getY());
        return (int) Math.signum(areaTwice);
    }

    private static boolean segmentIntersection(Translation2d a, Translation2d b, Translation2d c, Translation2d d) {
        // if c is left of ab and d is right of ab
        // or c is on ab or 
        boolean abLineIntersectsCDSegment;
        boolean cdLineIntersectsABSegment;
        {
        var cOrientation = leftRightOn(a, b, c);
        var dOrientation = leftRightOn(a, b, d);
        var oppositeSides = ((cOrientation == 1) && (dOrientation == -1)) // c left, d right
        || ((cOrientation == -1) && (dOrientation == 1)); // or c right, d left
        var cOn = cOrientation == 0;
        var dOn = dOrientation == 0;

        abLineIntersectsCDSegment = oppositeSides; // || cOn || dOn; // xor, so 
        }
        {
            var aOrientation = leftRightOn(c, d, a);
            var bOrientation = leftRightOn(c, d, b);
            var oppositeSides = ((aOrientation == 1) && (bOrientation == -1)) // c left, d right
            || ((aOrientation == -1) && (bOrientation == 1)); // or c right, d left
            var aOn = aOrientation == 0;
            var bOn = bOrientation == 0;
    
            cdLineIntersectsABSegment = oppositeSides; // || aOn || bOn; // xor, so 
        }
        return abLineIntersectsCDSegment && cdLineIntersectsABSegment;
    }
    
    public static boolean belowConstraints(Translation2d start, Translation2d end) {
        boolean clearLine = true;
        for (int i = 0; i<constraints.length; i++) {
            Translation2d bound = constraints[i%constraints.length];
            Translation2d boundNext = constraints[(i+1) % constraints.length];
            if (segmentIntersection(start, end, bound, boundNext)) {
                clearLine = false;
                // need to make clearLine false if any points below pq and between p and q

            }
            if (pointIsBelow(bound, start, end)) {
                clearLine = false;
            }

        }
        return clearLine;
    }

    public static boolean pointIsBelow(Translation2d testPoint, Translation2d start, Translation2d end) {
        if (pointIsBetween(testPoint, start, end)) {
            // orientation of point below the line
            // -1 if start x < end X (point is on the right)
            // 1 if start x > endX (point is on the left)
            // 0 if start and end are same X
            var wrongOrientation = (int) Math.signum(start.getX() - end.getX());
            return leftRightOn(start, end, testPoint) == wrongOrientation;
        }
        return false;
    }
    public static boolean pointIsBetween(Translation2d testPoint, Translation2d start, Translation2d end) {
        var testX = testPoint.getX();
        var startX = start.getX();
        var endX = end.getX();
        if (startX > endX) {
            return !(testX > startX || testX < endX);
        }
        else {
            return !(testX > endX || testX < startX);
        }
    }

    public static List<Translation2d> solvePath(Translation2d start, Translation2d end) {

        Translation2d currentPathTip = start;
        
        List<Translation2d> path = new LinkedList<Translation2d>();
        if (!belowConstraints(start, start) || !belowConstraints(end, end)) {
            return path;
        }
        List<Translation2d> candidates = new LinkedList<Translation2d>(Arrays.asList(constraints));
        path.add(start);
        for (int segment = 0; segment < 3; segment++ ){
            final Translation2d betweenTestEndpoint = currentPathTip;
            candidates.removeIf(
                (Translation2d testPoint)->
                    !pointIsBetween(testPoint, betweenTestEndpoint, end)
            );
            candidates.add(0, currentPathTip);
            candidates.add(end);
            if (currentPathTip.getX() >  end.getX()) {
                // if arg0 x > arg1 x, arg0 should be earlier, return -1
                candidates.sort((Translation2d arg0, Translation2d arg1)-> {return (int) Math.signum(arg1.getX() - arg0.getX());});
            }
            else {
                candidates.sort((Translation2d arg0, Translation2d arg1)-> {return (int) Math.signum(arg0.getX() - arg1.getX());});
            }
            Translation2d testPoint = end;
            for (int i = candidates.size() - 1; i > 0; i--) {
                testPoint = candidates.get(i);
                if(belowConstraints(currentPathTip, testPoint)) {
                    path.add(testPoint);
                    currentPathTip = testPoint;
                    break;
                }
            }
        }
            
        

        return path;
    }    /**
     * 
     * @param startPoint Translation2d of the start point
     * @param endPoint Translation2d of the desired end point
     * @return true if the end point is to the right of the start point, 
     * false if the end point is to the left of the start point
     */

    /*
    private boolean endPointSide(Translation2d startPoint, Translation2d endPoint) {
        return startPoint.getX() < endPoint.getX();
    }
    */


    /**
     * If the start point is to the right of the desired end point, 
     * switch the values of the start and end points
     * @param startPoint Traslation2d of the start point
     * @param endPoint Translation2d of the end point
     */
   
    /* 
    private void startPointOrient(Translation2d startPoint, Translation2d endPoint) {
        if(endPointSide(startPoint, endPoint)) {
            Translation2d startPointTemp = startPoint;
            startPoint = endPoint;
            endPoint = startPointTemp;
        }
    }

    */
    public static void main(String[] args) {
        System.out.println("Left (1): ");
        System.out.println(leftRightOn(new Translation2d(), new Translation2d(1,0), new Translation2d(1,1)));
        System.out.println("Extend (0): ");
        System.out.println(leftRightOn(new Translation2d(), new Translation2d(1,0), new Translation2d(2,0)));
        System.out.println("Turn In (0): ");
        System.out.println(leftRightOn(new Translation2d(), new Translation2d(1,0), new Translation2d(-1,0)));
        System.out.println("Right (-1)");
        System.out.println(leftRightOn(new Translation2d(), new Translation2d(1,0), new Translation2d(1, -1)));
        System.out.println();
        // testing SegmentIntersection
        System.out.println("Intersect (true)");
        System.out.println(segmentIntersection(new Translation2d(-1, 0), new Translation2d(1, 0)
        ,new Translation2d( 0,-1), new Translation2d( 0, 1)));
        System.out.println("Intersect Outside Segment (false)");
        System.out.println(segmentIntersection(new Translation2d(-1, 0), new Translation2d(1, 0)
        ,new Translation2d( 0,-1), new Translation2d( 0, -0.5)));
        System.out.println("Intersect Endpoint (false)");
        System.out.println(segmentIntersection(new Translation2d(-1, 0), new Translation2d(1, 0)
        ,new Translation2d( 0,-1), new Translation2d( 0, 0)));
        System.out.println("Intersect Parallel (false)");
        System.out.println(segmentIntersection(new Translation2d(-1, 0), new Translation2d(1, 0)
        ,new Translation2d( -1,-1), new Translation2d( 1, -1)));
        System.out.println("Intersect Identical (false)");
        System.out.println(segmentIntersection(new Translation2d(-1, 0), new Translation2d(1, 0)
        ,new Translation2d( -1,0), new Translation2d( 1, 0)));
        System.out.println("Intersect Collinear AB > CD (false)");
        System.out.println(segmentIntersection(new Translation2d(-1, 0), new Translation2d(1, 0)
        ,new Translation2d( -0.5,0), new Translation2d( 0.5, 0)));
        System.out.println("Intersect Collinear AB < CD (false)");
        System.out.println(segmentIntersection(new Translation2d(-1, 0), new Translation2d(1, 0)
        ,new Translation2d( -1.5,0), new Translation2d( 1.5, 0)));



    }



}