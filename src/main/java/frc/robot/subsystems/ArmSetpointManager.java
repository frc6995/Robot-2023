package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class ArmSetpointManager {
    Field2d VISUALIZER;
    // A list of constraint polygon vertices, going counterclockwise
    Translation2d[] constraintsPolyBack = {
        new Translation2d(1.567954,1.307665),
        new Translation2d(1.579967,0.530838),
        new Translation2d(3.794323,0.542851),
        new Translation2d(3.373875,0.695),
        new Translation2d(2.172597,1.728112)
    };

    ArrayList<Node> graph = new ArrayList<Node>();
    public ArmSetpointManager(Field2d VISUALIZER) {
        this.VISUALIZER = VISUALIZER;
        var poseList = new ArrayList<Pose2d> ();
        for (int i = 0; i < constraintsPolyBack.length; i++) {
            var translation = constraintsPolyBack[i];
            if(isVertexConcave(constraintsPolyBack, i)) {
                poseList.add(new Pose2d(translation, new Rotation2d(Math.PI)));
            } else {
                poseList.add(new Pose2d(translation, new Rotation2d()));
            }
        }
        VISUALIZER.getObject("constraints").setPoses(poseList);
        setupGraph(graph);
        

    }

    public void setupGraph(List<Node> graph, Node... additionalNodes) {
        for (int i = 0; i < constraintsPolyBack.length; i++) {
            var translation = constraintsPolyBack[i];
            if(isVertexConcave(constraintsPolyBack, i)) {
                graph.add(new Node(translation));
            }
        }
        for (var node : additionalNodes) {
            graph.add(node);
        }

        for (var a : graph) {
            for (var b : graph) {
                if(InLineOfSight(constraintsPolyBack, a.data, b.data)) {
                    a.addBranch(a.data.getDistance(b.data), b);
                    VISUALIZER.getObject(a.id + " " + b.id).setPoses(new Pose2d(a.data, new Rotation2d()),new Pose2d(b.data, new Rotation2d()));
                }
            }
        }
    }

    public void updateConstraintsPolygon() {
        var poses = VISUALIZER.getObject("constraints").getPoses();
        List<Translation2d> translationList = new ArrayList<>();
        for (var pose : poses) {
            translationList.add(pose.getTranslation());
        }
        Translation2d[] translationArr = new Translation2d[translationList.size()];
        translationList.toArray(translationArr);
        constraintsPolyBack = translationArr;


        var poseList = new ArrayList<Pose2d> ();
        for (int i = 0; i < constraintsPolyBack.length; i++) {
            var translation = constraintsPolyBack[i];
            if(isVertexConcave(constraintsPolyBack, i)) {
                poseList.add(new Pose2d(translation, new Rotation2d(Math.PI)));
            } else {
                poseList.add(new Pose2d(translation, new Rotation2d()));
            }
        }
        //VISUALIZER.getObject("constraints").setPoses(poseList);
    }

    public Translation2d[] calculatePath(Translation2d start, Translation2d end) {
        updateConstraintsPolygon();
        var localGraph = new ArrayList<Node>();
        Node startNode = new Node(start);
        Node endNode = new Node(end);
        setupGraph(localGraph, startNode, endNode);
        var aStarOutput = aStar(startNode, endNode);
        if (aStarOutput != null) { //path was actually generated
            List<Translation2d> pathPointList = new ArrayList<Translation2d>();
            Node n = aStarOutput;
            while(n.parent != null){
                pathPointList.add(n.data);
                n = n.parent;
            }
            pathPointList.add(n.data);
            Collections.reverse(pathPointList);
            Translation2d[] pathPointArr = new Translation2d[pathPointList.size()];
            pathPointList.toArray(pathPointArr);
            return pathPointArr;
        }
        else {
            return new Translation2d[] {endNode.data};
        }
    }

    public Node aStar(Node start, Node target){
        PriorityQueue<Node> closedList = new PriorityQueue<>();
        PriorityQueue<Node> openList = new PriorityQueue<>();
    
        start.f = start.g + start.calculateHeuristic(target);
        openList.add(start);
    
        while(!openList.isEmpty()){
            Node n = openList.peek();
            if(n == target){
                return n;
            }
    
            for(Node.Edge edge : n.neighbors){
                Node m = edge.node;
                double totalWeight = n.g + edge.weight;
    
                if(!openList.contains(m) && !closedList.contains(m)){
                    m.parent = n;
                    m.g = totalWeight;
                    m.f = m.g + m.calculateHeuristic(target);
                    openList.add(m);
                } else {
                    if(totalWeight < m.g){
                        m.parent = n;
                        m.g = totalWeight;
                        m.f = m.g + m.calculateHeuristic(target);
    
                        if(closedList.contains(m)){
                            closedList.remove(m);
                            openList.add(m);
                        }
                    }
                }
            }
    
            openList.remove(n);
            closedList.add(n);
        }
        return null;
    }
    
    // public static void printPath(Node target){
    //     Node n = target;
    
    //     if(n==null)
    //         return;
    
    //     List<Integer> ids = new ArrayList<>();
    
    //     while(n.parent != null){
    //         ids.add(n.id);
    //         n = n.parent;
    //     }
    //     ids.add(n.id);
    //     Collections.reverse(ids);
    
    //     for(int id : ids){
    //         System.out.print(id + " ");
    //     }
    //     System.out.println("");
    // }

    public static class Node implements Comparable<Node> {
        // Id for readability of result purposes
        private static int idCounter = 0;
        public int id;
        public final Translation2d data;
  
        // Parent in the path
        public Node parent = null;
  
        public List<Edge> neighbors;
  
        // Evaluation functions
        public double f = Double.MAX_VALUE;
        public double g = Double.MAX_VALUE;
        // Hardcoded heuristi
  
        Node(Translation2d data){
              this.data = data;
              this.id = idCounter++;
              this.neighbors = new ArrayList<>();
        }
  
        @Override
        public int compareTo(Node n) {
              return Double.compare(this.f, n.f);
        }
  
        public static class Edge {
              Edge(double weight, Node node){
                    this.weight = weight;
                    this.node = node;
              }
  
              public double weight;
              public Node node;
        }

        public void addBranch(double weight, Node node){
            Edge newEdge = new Edge(weight, node);
            neighbors.add(newEdge);
            
      }
  
        public double calculateHeuristic(Node target){
              return this.data.getDistance(target.data);
        }
  }


   //https://www.david-gouveia.com/pathfinding-on-a-2d-polygonal-map
    public static boolean lineSegmentsCross(Translation2d a, Translation2d b, Translation2d c, Translation2d d)
    {
        double denominator = ((b.getX() - a.getX()) * (d.getY() - c.getY())) - ((b.getY() - a.getY()) * (d.getX() - c.getX()));

        if (denominator == 0)
        {
            return false;
        }

        double numerator1 = ((a.getY() - c.getY()) * (d.getX() - c.getX())) - ((a.getX() - c.getX()) * (d.getY() - c.getY()));

        double numerator2 = ((a.getY() - c.getY()) * (b.getX() - a.getX())) - ((a.getX() - c.getX()) * (b.getY() - a.getY()));

        if (numerator1 == 0 || numerator2 == 0)
        {
            return false;
        }

        double r = numerator1 / denominator;
        double s = numerator2 / denominator;

        return (r > 0 && r < 1) && (s > 0 && s < 1);
    }

    public static boolean isVertexConcave(Translation2d[] vertices, int vertex)
    {
        Translation2d current = vertices[vertex];
        Translation2d next = vertices[(vertex + 1) % vertices.length];
        Translation2d previous = vertices[vertex == 0 ? vertices.length - 1 : vertex - 1];

        Translation2d edge1 = new Translation2d(current.getX() - previous.getX(), current.getY() - previous.getY());
        Translation2d edge2 = new Translation2d(next.getX() - current.getX(), next.getY() - current.getY());

        double cross = (edge1.getX() * edge2.getY()) - (edge1.getY() * edge2.getX());

        return cross < 0;
    }

    public static boolean inside(Translation2d[] polygon, Translation2d position) {
    Translation2d point = position;

    boolean inside = false;

    // Must have 3 or more edges
    if (polygon.length < 3) return false;

    Translation2d oldPoint = polygon[polygon.length - 1];

    for (int i = 0; i < polygon.length; i++)
    {
        Translation2d newPoint = polygon[i];
        Translation2d left;
        Translation2d right;
        if (newPoint.getX() > oldPoint.getX())
        {
            left = oldPoint;
            right = newPoint;
        }
        else
        {
            left = newPoint;
            right = oldPoint;
        }

        if (left.getX() < point.getX() && point.getX() <= right.getX() && (point.getY() - left.getY()) * (right.getX() - left.getX()) < (right.getY() - left.getY()) * (point.getX() - left.getX()))
            inside = !inside;

        oldPoint = newPoint;
    }

    return inside;
}

boolean InLineOfSight(Translation2d[] polygon, Translation2d start, Translation2d end)
{
  // Not in LOS if any of the ends is outside the polygon
  //if (!inside(polygon, start) || !inside(polygon, end)) return false;

  // In LOS if it's the same start and end location
  if (start.getDistance(end) < 0.001) return true;

  // Not in LOS if any edge is intersected by the start-end line segment
    for (int i = 0; i < polygon.length; i++)
      if (lineSegmentsCross(start, end, polygon[i], polygon[(i+1)%polygon.length]))
        return false;

  // Finally the middle point in the segment determines if in LOS or not
  return inside(polygon, new Translation2d((start.getX() + end.getX())/2, (start.getY() + end.getY())/2));
}

}

