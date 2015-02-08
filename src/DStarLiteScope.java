import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.Rectangle2D;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Scanner;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

/**
 * @modified Aakash Parikh Justin Lin Kevin Wang
 *
 *           Copyright (C) 2012 Daniel Beard
 *
 */
public class DStarLiteScope implements java.io.Serializable {
	private static ArrayList<Obstacles> obstaclesList = new ArrayList<Obstacles>();

	private static List<State> traveledPath = new ArrayList<State>();
	// Private Member variables

	static int sX;
	static int sY ;
	static int eX ;
	static int eY ;
	static int cX ;
	static int cY ;
	static long begin;
	static final long MAX_TIME = 5000;
	State prevCur = new State();
	
	private List<State> path = new ArrayList<State>();
	private double C1;
	private double k_m;
	private State s_start = new State();
	private State s_goal = new State();
	private State s_last = new State();
	private State s_current = new State();
	private int maxSteps;
	private PriorityQueue<State> openList = new PriorityQueue<State>();
	// Change back to private****
	public HashMap<State, CellInfo> cellHash = new HashMap<State, CellInfo>();
	private HashMap<State, Float> openHash = new HashMap<State, Float>();

	// Constants
	private static double M_SQRT2 = Math.sqrt(2.0);

	private static final double maxVisionRange = 30; //note: min slightly more than sqrt2
	private static final int MAXTIME = 5000;

	// Default constructor
	public DStarLiteScope() {
		maxSteps = 80000;
		C1 = 1;
	}


	/*
	 * Initialize Method
	 * 
	 * @params start and goal coordinates
	 */
	public void init(int sX, int sY, int gX, int gY, int cX, int cY) {
		cellHash.clear();
		path.clear();
		openHash.clear();
		while (!openList.isEmpty())
			openList.poll();

		k_m = 0;

		s_start.x = sX;
		s_start.y = sY;
		s_goal.x = gX;
		s_goal.y = gY;
		CellInfo tmp = new CellInfo();
		tmp.g = 0;
		tmp.rhs = 0;
		tmp.cost = C1;

		cellHash.put(s_goal, tmp);

		tmp = new CellInfo();
		tmp.g = tmp.rhs = heuristic(s_start, s_goal);
		tmp.cost = C1;
		cellHash.put(s_start, tmp);
		s_start = calculateKey(s_start);

		s_last = s_start;
		
		s_current.x = s_start.x;
		s_current.y = s_start.y;

	}

	/*
	 * CalculateKey(state u) As per [S. Koenig, 2002]
	 */
	private State calculateKey(State u) {
		double val = Math.min(getRHS(u), getG(u));

		u.k.setFirst(val + heuristic(u, s_start) + k_m);
		u.k.setSecond(val);

		return u;
	}

	/*
	 * Returns the rhs value for state u.
	 */
	private double getRHS(State u) {

		if (u == s_goal)
			return 0;

		// if the cellHash doesn't contain the State u
		if (cellHash.get(u) == null)
			return heuristic(u, s_goal);
		
		if (cellHash.get(u).cost < 0 ) {
			return Double.POSITIVE_INFINITY;
		}
		return cellHash.get(u).rhs;
	}

	/*
	 * Returns the g value for the state u.
	 */
	private double getG(State u) {
		// if the cellHash doesn't contain the State u
		if (cellHash.get(u) == null)
			return heuristic(u, s_goal);
		return cellHash.get(u).g;
	}

	/*
	 * Pretty self explanatory, the heuristic we use is the 8-way distance
	 * scaled by a constant C1 (should be set to <= min cost)
	 */
	private double heuristic(State a, State b) {
		return eightCondist(a, b) * C1;
	}

	/*
	 * Returns the 8-way distance between state a and state b
	 */
	private double eightCondist(State a, State b) {
		double temp;
		double min = Math.abs(a.x - b.x);
		double max = Math.abs(a.y - b.y);
		if (min > max) {
			temp = min;
			min = max;
			max = temp;
		}
		return ((M_SQRT2 - 1.0) * min + max);

	}

	public boolean calculatePath() {
		System.out.println("START PATH CALC");
		path.clear();

		int res = computeShortestPath();
		if (res < 0) {
			System.out.println("No Path to Goal");
			return false;
		}

		LinkedList<State> n = new LinkedList<State>();
		State cur = s_start;


		if (getG(s_start) == Double.POSITIVE_INFINITY) {
			System.out.println("No Path to Goal");
			return false;
		}
		boolean running = true;
		// while (s_current.neq(s_goal) && (running)) {
		//(System.currentTimeMillis() - begin < MAXTIME)&& 
		while (cur.neq(s_goal) && trueDist(s_start, cur) < maxVisionRange-.0001) {
			System.out.println("s_start: "+s_start.x+","+ s_start.y+ " cur:"+cur.x+","+cur.y+ " dist:"+trueDist(s_start, cur));
			path.add(cur);
			
			prevCur.x = cur.x;
			prevCur.y = cur.y;
			System.out.println("PREV CUR START:" + prevCur.x+","+prevCur.y);

			traveledPath.add(new State(cur));
			// System.out.println(traveledPath.toString());
			n = new LinkedList<State>();
			n = getSucc(cur);

			if (n.isEmpty()) {
				System.out.println("No Path to Goal");
				return false;
			}

			double cmin = Double.POSITIVE_INFINITY;
			double tmin = 0;
			State smin = new State();

			for (State i : n) {
				double val = getRHS(i);
				//cost(cur, i);
				double val2 = trueDist(i, s_goal) + trueDist(s_start, i);
				//val += getG(i);

				if (close(val, cmin)) {
					if (tmin > val2) {
						tmin = val2;
						cmin = val;
						smin = i;
					}
				} else if (val < cmin) {
					tmin = val2;
					cmin = val;
					smin = i;
				}
			}
			n.clear();
			cur = new State(smin);

			// cur = smin;
		}

		if (cur.eq(s_goal)) {
			s_current = cur;
			traveledPath.add(new State(s_goal));
		} else if(trueDist(s_start,cur) < maxVisionRange){
			s_current.x = cur.x;
			s_current.y = cur.y;
			System.out.println("UPDATE START" + cur.x+","+cur.y);
			updateStart(cur.x, cur.y);
		}else {
			System.out.println("UPDATE START PREV" + prevCur.x+","+prevCur.y);
			updateStart(prevCur.x, prevCur.y);
		}

		 //updateObstacles(obstaclesList);

		return true;
	}

	/*
	 * As per [S. Koenig,2002] except for two main modifications: 1. We stop
	 * planning after a number of steps, 'maxsteps' we do this because this
	 * algorithm can plan forever if the start is surrounded by obstacles 2. We
	 * lazily remove states from the open list so we never have to iterate
	 * through it.
	 */
	private int computeShortestPath() {
		//System.out.println("in compute Shortest Path");
		LinkedList<State> s = new LinkedList<State>();

		if (openList.isEmpty())
			return 1;

		int k = 0;
		
		while ( (!openList.isEmpty())&& (openList.peek().lt(s_start = calculateKey(s_start))|| (getRHS(s_start) != getG(s_start)))){

			if (k++ > maxSteps) {
				System.out.println("At maxsteps");
				return -1;
			}

			State u;

			boolean test = (getRHS(s_start) != getG(s_start));

			// lazy remove
			while (true) {
				if (openList.isEmpty())
					return 1;
				u = openList.poll();
				//System.out.println("OPEN LIST SIZE:"+openList.size());

				if (!isValid(u))
					continue;
				if (!(u.lt(s_start)) && (!test))
					return 2;
				break;
			}

			openHash.remove(u);

			State k_old = new State(u);

			/*if (k_old.lt(calculateKey(u))) { // u is out of date
				insert(u);
				System.out.println("CHOICE 1");
			} else */
			if (getG(u) > getRHS(u)) { // needs update (got better)
				//System.out.println("CHOICE 2");
				setG(u, getRHS(u));
				s = getPred(u);
				for (State i : s) {
					updateVertex(i);
				}
			} else if(getG(u) < getRHS(u)){//, state has got worse
				//System.out.println("CHOICE 3");
				setG(u, Double.POSITIVE_INFINITY);
				s = getPred(u);

				for (State i : s) {
					updateVertex(i);
				}
				updateVertex(u);
			}
		} // while
		return 0;
	}

	/*
	 * Returns a list of successor states for state u, since this is an 8-way
	 * graph this list contains all of a cells neighbours. Unless the cell is
	 * occupied, in which case it has no successors.
	 */
	private LinkedList<State> getSucc(State u) {
		LinkedList<State> s = new LinkedList<State>();
		State tempState;
		if (occupied(u))
			return s;
		// Generate the successors, starting at the immediate right,
		// Moving in a clockwise manner
		tempState = new State(u.x + 1, u.y, new Pair(-1.0, -1.0));
		s.addFirst(tempState);
		tempState = new State(u.x + 1, u.y + 1, new Pair(-1.0, -1.0));
		s.addFirst(tempState);
		tempState = new State(u.x, u.y + 1, new Pair(-1.0, -1.0));
		s.addFirst(tempState);
		tempState = new State(u.x - 1, u.y + 1, new Pair(-1.0, -1.0));
		s.addFirst(tempState);
		tempState = new State(u.x - 1, u.y, new Pair(-1.0, -1.0));
		s.addFirst(tempState);
		tempState = new State(u.x - 1, u.y - 1, new Pair(-1.0, -1.0));
		s.addFirst(tempState);
		tempState = new State(u.x, u.y - 1, new Pair(-1.0, -1.0));
		s.addFirst(tempState);
		tempState = new State(u.x + 1, u.y - 1, new Pair(-1.0, -1.0));
		s.addFirst(tempState);

		return s;
	}

	/*
	 * Returns a list of all the predecessor states for state u. Since this is
	 * for an 8-way connected graph, the list contains all the neighbours for
	 * state u. Occupied neighbors are not added to the list
	 */
	private LinkedList<State> getPred(State u) {
		LinkedList<State> s = new LinkedList<State>();
		State tempState;
		//System.out.println("DAUGHTER NODE:" + u.x+","+u.y);
		
		tempState = new State(u.x + 1, u.y, new Pair(-1.0, -1.0));
		if (!occupied(tempState)){
			s.addFirst(tempState);
			//System.out.println("NODE "+ tempState.x +", "+tempState.y);
		}
			
			
		tempState = new State(u.x + 1, u.y + 1, new Pair(-1.0, -1.0));
		if (!occupied(tempState)){

			s.addFirst(tempState);
		//System.out.println("NODE "+ tempState.x +", "+tempState.y);			
		}
		
		tempState = new State(u.x, u.y + 1, new Pair(-1.0, -1.0));
		if (!occupied(tempState)){
			s.addFirst(tempState);
			//System.out.println("NODE "+ tempState.x +", "+tempState.y);	
		}
			
		
		tempState = new State(u.x - 1, u.y + 1, new Pair(-1.0, -1.0));
		if (!occupied(tempState)){
			s.addFirst(tempState);
		//System.out.println("NODE "+ tempState.x +", "+tempState.y);
		}
		
		tempState = new State(u.x - 1, u.y, new Pair(-1.0, -1.0));
		if (!occupied(tempState)){
			s.addFirst(tempState);
		//System.out.println("NODE "+ tempState.x +", "+tempState.y);
		}
		
		tempState = new State(u.x - 1, u.y - 1, new Pair(-1.0, -1.0));
		
		if (!occupied(tempState)){
			s.addFirst(tempState);
			//System.out.println("NODE "+ tempState.x +", "+tempState.y);
		}
		tempState = new State(u.x, u.y - 1, new Pair(-1.0, -1.0));
		if (!occupied(tempState)){
			s.addFirst(tempState);
		//System.out.println("NODE "+ tempState.x +", "+tempState.y);
		}
		tempState = new State(u.x + 1, u.y - 1, new Pair(-1.0, -1.0));
		if (!occupied(tempState)){
			s.addFirst(tempState);
		//System.out.println("NODE "+ tempState.x +", "+tempState.y);
		}
		return s;
	}

	/*
	 * Update the position of the agent/robot. This does not force a
	 * calculatePath.
	 */
	public void updateStart(int x, int y) {
		s_start.x = x;
		s_start.y = y;

	// k_m += heuristic(s_last, s_start); //COMMENTED THIS OUT

		s_start = calculateKey(s_start);
		s_last = s_start;

	}

	/*
	 * This is somewhat of a hack, to change the position of the goal we first
	 * save all of the non-empty nodes on the map, clear the map, move the goal
	 * and add re-add all of the non-empty cells. Since most of these cells are
	 * not between the start and goal this does not seem to hurt performance too
	 * much. Also, it frees up a good deal of memory we are probably not going
	 * to use.
	 */
	public void updateGoal(int x, int y) {
		List<Pair<iPoint2, Double>> toAdd = new ArrayList<Pair<iPoint2, Double>>();
		Pair<iPoint2, Double> tempPoint;

		for (Map.Entry<State, CellInfo> entry : cellHash.entrySet()) {
			if (!close(entry.getValue().cost, C1)) {
				tempPoint = new Pair(new iPoint2(entry.getKey().x,
						entry.getKey().y), entry.getValue().cost);
				toAdd.add(tempPoint);
			}
		}

		cellHash.clear();
		openHash.clear();

		while (!openList.isEmpty())
			openList.poll();

		k_m = 0;

		s_goal.x = x;
		s_goal.y = y;

		CellInfo tmp = new CellInfo();
		tmp.g = tmp.rhs = 0;
		tmp.cost = C1;

		cellHash.put(s_goal, tmp);

		tmp = new CellInfo();
		tmp.g = tmp.rhs = heuristic(s_start, s_goal);
		tmp.cost = C1;
		cellHash.put(s_start, tmp);
		s_start = calculateKey(s_start);

		s_last = s_start;

		Iterator<Pair<iPoint2, Double>> iterator = toAdd.iterator();
		while (iterator.hasNext()) {
			tempPoint = iterator.next();
			updateCell(tempPoint.first().x, tempPoint.first().y,
					tempPoint.second());
		}

	}

	/*
	 * As per [S. Koenig, 2002]
	 */
	private void updateVertex(State u) {
		LinkedList<State> s = new LinkedList<State>();

		if (u.neq(s_goal)) {
			s = getSucc(u);
			double tmp = Double.POSITIVE_INFINITY;
			double tmp2;

			for (State i : s) {
				tmp2 = getG(i) + cost(u, i);
				if (tmp2 < tmp)
					tmp = tmp2;
			}
			if (!close(getRHS(u), tmp))
				setRHS(u, tmp);
		}

		if (!close(getG(u), getRHS(u)))
			insert(u);
		// System.out.println("UPDATE VERTEX" + u.x + "," + u.y);
	}

	/*
	 * Returns true if state u is on the open list or not by checking if it is
	 * in the hash table.
	 */
	private boolean isValid(State u) {
		if (openHash.get(u) == null)
			return false;
		if (!close(keyHashCode(u), openHash.get(u)))
			return false;
		return true;
	}

	/*
	 * Sets the G value for state u
	 */
	private void setG(State u, double g) {
		makeNewCell(u);
		cellHash.get(u).g = g;
	}

	/*
	 * Sets the rhs value for state u
	 */
	private void setRHS(State u, double rhs) {
		makeNewCell(u);
		cellHash.get(u).rhs = rhs;
	}

	/*
	 * Checks if a cell is in the hash table, if not it adds it in.
	 */
	private void makeNewCell(State u) {
		if (cellHash.get(u) != null)
			return;
		CellInfo tmp = new CellInfo();
		tmp.g = tmp.rhs = heuristic(u, s_goal);
		tmp.cost = C1;
		cellHash.put(u, tmp);
	}

	public void markImpassable(int x, int y) {
		State u = new State();
		u.x = x;
		u.y = y;

		CellInfo tmp = new CellInfo();
		tmp.g = tmp.rhs = heuristic(u, s_goal);
		tmp.cost = -1;
		cellHash.put(u, tmp);
		setG(u, Double.POSITIVE_INFINITY);

	}

	/*
	 * updateCell as per [S. Koenig, 2002]
	 */
	public void updateCell(int x, int y, double val) {
		State u = new State();
		u.x = x;
		u.y = y;

		if ((u.eq(s_start)) || (u.eq(s_goal)))
			return;

		makeNewCell(u);
		cellHash.get(u).cost = val;
		/*
		if(val < 0){
			cellHash.get(u).markAsInvalid();
		}*/
		updateVertex(u);
	}

	/*
	 * Inserts state u into openList and openHash
	 */
	private void insert(State u) {
		// iterator cur
		float csum;

		u = calculateKey(u);
		// cur = openHash.find(u);
		csum = keyHashCode(u);

		// return if cell is already in list. TODO: this should be
		// uncommented except it introduces a bug, I suspect that there is a
		// bug somewhere else and having duplicates in the openList queue
		// hides the problem...
		// if ((cur != openHash.end()) && (close(csum,cur->second))) return;

		openHash.put(u, csum);
		openList.add(u);
	}

	/*
	 * Returns the key hash code for the state u, this is used to compare a
	 * state that has been updated
	 */
	private float keyHashCode(State u) {
		return (float) (u.k.first() + 1193 * u.k.second());
	}

	/*
	 * Returns true if the cell is occupied (non-traversable), false otherwise.
	 * Non-traversable are marked with a cost < 0
	 */
	private boolean occupied(State u) {
		// if the cellHash does not contain the State u
		if (cellHash.get(u) == null)
			return false;
		if(!cellHash.get(u).valid){
			return false;
		}
		return (cellHash.get(u).cost < 0);
	
		/*
		if(u.x > eX){
			System.out.println("2");
			return true;
		}else if (u.y > eY){
			System.out.println("3");
			return true;
		}else if (u.x<sX){
			System.out.println("4");
			return true;
		}else if (u.y < sY){
			System.out.println("5");
			return true;
		}else if(isValid(u)){
			if (cellHash.get(u).cost < 0){
			System.out.println("6");
			return false;
			}else{
				return false;
			}
		}else{
			System.out.println("7");
			return false;
		}*/
	}

	/*
	 * Euclidean cost between state a and state b
	 */
	private double trueDist(State a, State b) {
		float x = a.x - b.x;
		float y = a.y - b.y;
		return Math.sqrt(x * x + y * y);
	}

	/*
	 * Returns the cost of moving from state a to state b. This could be either
	 * the cost of moving off state a or onto state b, we went with the former.
	 * This is also the 8-way cost.
	 */
	private double cost(State a, State b) {
		if (cellHash.get(a) != null && cellHash.get(b) != null) {
			if (cellHash.get(a).cost < 0 || cellHash.get(b).cost < 0) {
				return Double.POSITIVE_INFINITY;
			}
		}
		int xd = Math.abs(a.x - b.x);
		int yd = Math.abs(a.y - b.y);
		double scale = 1;

		if (xd + yd > 1)
			scale = M_SQRT2;

		if (cellHash.containsKey(a) == false)
			return scale * C1;
		return scale * cellHash.get(a).cost;
	}

	/*
	 * Returns true if x and y are within 10E-5, false otherwise
	 */
	private boolean close(double x, double y) {
		if (x == Double.POSITIVE_INFINITY && y == Double.POSITIVE_INFINITY)
			return true;
		return (Math.abs(x - y) < 0.00001);
	}

	public List<State> getPath() {
		return path;
	}

	public void createWalls(int x, int y) { // bottomleftmost point
		for (int i = -1; i < x + 2; i++) {
			updateCell(i, y + 1, -1);
			updateCell(i, -1, -1);
		}
		for (int i = -1; i < y + 2; i++) {
			updateCell(x + 1, i, -1);
			updateCell(-1, i, -1);
		}
	}

	public static void main(String[] args) {
		DStarLiteScope pf = new DStarLiteScope();

		// Adding obstacles an creating the maze
		Scanner sc = null;
		try {
			sc = new Scanner(new File("largeMaze.txt"));
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		begin = System.currentTimeMillis();
		sX = sc.nextInt();
		sY = sc.nextInt();
		eX = sc.nextInt();
		eY = sc.nextInt();
		cX = sc.nextInt();
		cY = sc.nextInt();
		pf.init(sX, sY, eX, eY, cX, cY);
		pf.createWalls(cX, cY);
		// pf.createWalls(10, 10); // (xmax, //ymax)
		int numOfObstacles = sc.nextInt();
		for (int i = 0; i < numOfObstacles; i++) {
			while (sc.hasNext()) {
				int x = sc.nextInt();
				int y = sc.nextInt();
				Obstacles temp = new Obstacles(x, y);
				obstaclesList.add(temp);
			}
		}
		boolean running = true;
		while (pf.s_current.neq(pf.s_goal) && (running)) {
			/*
			 * if((System.currentTimeMillis() - begin > MAXTIME)){
			 * System.out.println("No Path to Goal"); break; }
			 */
			pf.updateObstacles(obstaclesList);
			//System.out.println("CALL CALC");
			running = pf.calculatePath();
		}

		// Time the replanning

		// update obstacles
		// move start
		// recalculate path

		// pf.updateGoal(12, 4);

		List<State> path = pf.getPath();

		// Calculate distance
		double distance = 0;
		int prevX = sX;
		int prevY = sY;
		Canvas c = pf.new Canvas((ArrayList<State>) traveledPath, obstaclesList);
		c.doDrawing();
		
		for (State i : pf.traveledPath) {
			
			
			
			System.out.println("x: " + i.x + " y: " + i.y);
			distance += Math.sqrt(Math.pow(i.x - prevX, 2)
					+ Math.pow(i.y - prevY, 2));
			prevX = i.x;
			prevY = i.y;
		}
		System.out.println();
		System.out.println("Start node: (" + sX + "," + sY + ")");
		System.out.println("End node: (" + eX + "," + eY + ")");
		/*
		System.out.println("Number of Obstacles: " + numOfObstacles + ": ");
		for (int i = 0; i < obstaclesList.size(); i++) {
			Obstacles o = obstaclesList.get(i);
			System.out.println(o);
		}*/
		//System.out.println("Steps = " + (pf.traveledPath.size() - 1));
		long end = System.currentTimeMillis();
		System.out.println("Time: " + (end - begin) + "ms");
		System.out.println("Distance: " + distance);

	}

	public void updateObstacles(ArrayList<Obstacles> obstaclesList) {
		for (Obstacles o : obstaclesList) {
			System.out.println("DIST:"+(Math.sqrt(Math.pow(o.x - s_start.x, 2)
					+ Math.pow(o.y - s_start.y, 2)) ));
			if (Math.sqrt(Math.pow(o.x - s_start.x, 2)
					+ Math.pow(o.y - s_start.y, 2)) <= maxVisionRange) {
				System.out.println("OBS: " + o.x+","+o.y);
				// setG(, Double.POSITIVE_INFINITY);
				//markImpassable(o.x, o.y);
				updateCell(o.x, o.y, -1);
				
			}
		}
	}
	
	
	 class Canvas extends JFrame{
		 ArrayList <State>traveledPath = new ArrayList<State>();
		 ArrayList <Obstacles> obstaclesList = new ArrayList<Obstacles>();
		public Canvas(ArrayList <State> l, ArrayList <Obstacles> o){
			traveledPath = l;
			obstaclesList = o;
			initUI(traveledPath);
		}
		
		private void initUI(ArrayList <State> l){
			setTitle("Maze");
			setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			
			add(new Surface(traveledPath, obstaclesList));
			
			setSize((int)(Surface.width * (cX+2)), (int)(Surface.height * (cY+2.5)));
			setLocationRelativeTo(null);
		}
		public void doDrawing(){
			SwingUtilities.invokeLater(new Runnable(){
				@Override
				public void run(){
					setVisible(true);
				}
			});
		}
		
	}
	
	static class Surface extends JPanel{

		ArrayList<State> traveledList = new ArrayList<State> ();
		ArrayList <Obstacles> obstaclesList = new ArrayList<Obstacles>();
		public Surface(ArrayList<State> l, ArrayList<Obstacles> o){
			super();
			traveledList = l;
			obstaclesList = o;
			setSize(Surface.width * (cX+1), Surface.height * (cY+1));
			
		}
		
		static int width = 25;
		static int height = 25;
		public void paintRect (Graphics g, int x, int y, float rectWidth, float rectHeight, Color c){
			Graphics2D g2 = (Graphics2D)g;
			g2.setColor(c);
			Rectangle2D.Double rect = new Rectangle2D.Double(x, y, rectWidth, rectHeight);
			
			g2.draw(rect);
			g2.fill(rect);
		}
		
		
		
		@Override
		public void paintComponent(Graphics g){
			super.paintComponent(g);
			for(int i = 0; i < cX + 1; i++){
				for(int j =0; j<cY + 1; j++){
					paintRect(g, i*width, j*height, width, height, Color.BLACK);
				}
			}
			for(int i = 0; i < traveledList.size(); i++){
				paintRect(g, traveledList.get(i).x*width,traveledList.get(i).y*height, width, height, Color.MAGENTA);
				//System.out.println("Add rect" + traveledList.get(i).x +","+traveledList.get(i).y);
			}
			for(int i = 0; i < obstaclesList.size(); i++){
				paintRect(g, obstaclesList.get(i).x*width,obstaclesList.get(i).y*height, width, height, Color.RED);
				//System.out.println("Add rect" + obstaclesList.get(i).x +","+obstaclesList.get(i).y);
			}
			
		}
		/*
		int i = 0;
		setVisible(true);
		
		while(i < traveledPath.size()){

			Surface.paintRect(traveledPath.get(i).x, traveledPath.get(i).y,
					25,25);
			i++;
		}*/

	}
}



class CellInfo implements java.io.Serializable {
	public double g = 0;
	public double rhs = 0;
	public double cost = 0;
	public boolean valid = true;
	
	public void markAsInvalid(){
		valid = false;
		g= Double.POSITIVE_INFINITY;
	}
}

class iPoint2 {
	public int x;
	public int y;

	// default constructor
	public iPoint2() {
		this.x = 0;
		this.y = 0;
	}

	// overloaded constructor
	public iPoint2(int x, int y) {
		this.x = x;
		this.y = y;
	}
}