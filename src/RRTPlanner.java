/**
 * This algorithm is from the following paper
 * Steven M. LaValle and James  Kuffner Jr.,
 * Randomized Kinodynamic Planning, 
 * The International Journal of Robotics Research 20 (5), pp. 378–400.
 * http://dx.doi.org/10.1177/02783640122067453
 */

package assignment_motion_planning;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.List;
import java.util.LinkedList;

import javafx.geometry.Point2D;
import javafx.util.Pair;


public class RRTPlanner extends MotionPlanner {
    private static final double DEFAULT_DELTA = 0.01;  // Duration for the control

    protected List<Vector> configs;
    protected Map<Vector, Edge> parents;

    public class Edge {
      public Pair<Point2D, Point2D> location;
      public Vector parent;
      public Vector child;
      public Vector control;
      public double duration;

      public Edge(Vector parent, Vector child, Vector c, double d) {
        this.parent = parent;
        this.child = child;
        this.control = c;
        this.duration = d;
        this.location = new Pair<Point2D, Point2D>(new Point2D(parent.get(0), parent.get(1)),
                                  new Point2D(child.get(0), child.get(1)));
      }
    }

    /**
     * Constructor 
     * @param environment the workspace
     * @param robot       the robot
     */
    public RRTPlanner(Environment environment, Robot robot) {
        super(environment, robot);
    }
    
    @Override
    public List<Pair<Point2D, Point2D>> getEdges() {
      LinkedList<Pair<Point2D, Point2D>> result = new LinkedList<Pair<Point2D, Point2D>>();

      for (Edge e : parents.values()) {
        if (e != null)
          result.add(e.location);
      }

      return result;
    }
    
    @Override
    public int getSize() {
        // YOU WILL WRITE THIS METHOD
        return 0;
    }

    @Override
    protected void setup() {
      configs = new LinkedList<Vector>();
      parents = new HashMap<Vector, Edge>();
    }
    
    @Override
    protected void growMap(int K) {
      Vector qrand, qnear;
      Vector control;
      int maxTries = getRobot().getControls().size();
      int tries = 0;

      // Initialize the Map with the Start Condition
      if (! getEnvironment().isValidConfiguration(getRobot(), getStart())) {
        System.out.println("Invalid Start Condition");
        return;
      }
      configs.add(getStart());
      parents.put(getStart(), null);

      for (int i = 0; i < K; i++) {
        qrand = getRobot().getRandomConfiguration(getEnvironment(), random);
        qnear = nearestNeighbor(configs, qrand);
        
        while (! newConf(qnear, DEFAULT_DELTA) && ++tries <= maxTries)
          ;

        tries = 0;
      }
    }

    /**
     * Generate a new configuration from a configuration and insert it
     * @param qnear    the beginning configuration of the random motion
     * @param duration the duration of the random motion
     * @return true if one new configuration is inserted, and false otherwise
     */
    @SuppressWarnings("boxing")
    private boolean newConf(Vector qnear, double duration) {
      Vector qnew;
      Vector control = getRobot().getRandomControl(random);
      
      if (getEnvironment().isValidMotion(getRobot(),
                                          qnear,
                                          new Trajectory(control, duration),
                                          RESOLUTION)) {
        // Apply the valid motion to get the next configuration
        qnew = getRobot().move(qnear, control, duration);
        
        // Add the new vector to the list and the map
        if (! parents.containsKey(qnew)) {
          configs.add(qnew);
          parents.put(qnew, new Edge(qnear, qnew, control, duration));
          return true;
        } else {
          return false;
        }
      } else {
        return false;
      }
    }
    
    @SuppressWarnings("boxing")
    @Override
    protected Trajectory findPath() {
      LinkedList<Edge> chain;       // The Backchain
      Vector curr;                  // Current configuration
      Trajectory result;            // The forward-directional trajectory object


      // Starting place is the node in list closest to the goal
      curr = nearestNeighbor(configs, getGoal());
      chain = new LinkedList<Edge>();
      System.out.println("Closest Vertex: " + curr);

      // Create a linked list of edges by backchaining 
      if (parents.get(curr) == null)
          System.out.println("Curr is null: " + curr + ", " + parents.get(curr));

      while(parents.get(curr) != null) {
        //System.out.println("Curr = " + curr);
        chain.addFirst(parents.get(curr));
        curr = parents.get(curr).parent;
        //System.out.println("Next Curr = " + curr);
      }

      //System.out.println("Edge List Made");

      // Now loop forward through the chain to make the result
      result = new Trajectory();
      for (Edge e : chain)
        result.addControl(e.control, e.duration);

      //System.out.println("Trajectory Made" + result);
      return result;
    }

    @Override
    protected void reset() {
      configs = null;
      parents = null;
    }

}
