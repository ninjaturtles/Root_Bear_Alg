
//import statements
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Scanner;


/**
 * Root bear Algorithm - Can the bear cross the canal
 * 
 * @author Akanksha
 * @since December 02, 2017
 *
 */
public class RootBear {

	//global variables
	private static final int SOURCE = 0;
	private Queue<Integer> queue;
	private int[] visited;
	private static int diameter;

	public static void main(String args[]) {

		Vertex northWall;
		Vertex southWall;
		int trees; //number of tress in the problem
		RootBear rb = new RootBear();
		Graph graph;
		int[] firstLine = new int[4];

		Scanner sc = new Scanner(System.in);
		String line = sc.nextLine();
		Scanner lineScanner = new Scanner(line);

		//reads the first 4 arguments on the line 
		for(int i = 0; i< 4; i++) {
			firstLine[i] = lineScanner.nextInt();
		}

		diameter = firstLine[2];
		trees = firstLine[3];
		graph = rb.new Graph(trees + 2);

		northWall = rb.new Vertex(0,firstLine[0]);
		southWall = rb.new Vertex(0, firstLine[1]);		
		graph.addVertex(northWall); //add vertex north
		graph.addVertex(southWall); //add vertex south

		Scanner vertexScanner;
		
		//reads all the x and y position of the tress and adds to graph
		for(int i = 0; i < firstLine[3]; i++) {
			line = sc.nextLine();
			vertexScanner = new Scanner(line);
			int x = vertexScanner.nextInt();
			int y = vertexScanner.nextInt();
			Vertex vertex = rb.new Vertex(x,y);
			graph.addVertex(vertex);
			if(i == firstLine[3]) {
				vertexScanner.close();
			}
		}
		
		lineScanner.close();
		sc.close();
		
		System.out.println("--------------------------");
		rb.findAllEdges(graph, northWall, southWall);

		rb.bfs(graph.adjacencyMatrix,SOURCE); //run BFS on the graph
		
		if(rb.isConnected()) {
			System.out.println("Can pass?: NO");
		}else {
			System.out.println("Can pass?: YES");
		}

	} // main

	/**
	 * Runs the BFS algorithm from one wall
	 * 
	 * @param adj_matrix - adjency matrix
	 * @param source - index
	 */
	public void bfs(int adj_matrix[][], int source) {
		queue = new LinkedList<Integer>();
		int nodes = adj_matrix[source].length -1;
		visited = new int[nodes +1];
		int i, element;

		visited[source] = 1;
		queue.add(source);

		while(!queue.isEmpty()) {
			element = queue.remove();
			i=element;
			while(i<=nodes) {
				if(adj_matrix[element][i] == 1 && visited[i] == 0) {
					queue.add(i);
					visited[i] = 1;
				}
				i++;
			}
		}
	}

	/**
	 * Finds all the edges given the points. Only adds an edge if distance between them
	 * is less than the diameter of the bear.
	 * 
	 * @param g - graph G
	 * @param northWall - vertex of the north wall
	 * @param southWall = vertex of the south wall
	 */
	public void findAllEdges(Graph g,Vertex northWall, Vertex southWall ) {
		for(int i = 0; i < g.vertices.size(); i++) {
			for(int j = 0; j<g.vertices.size(); j++) {
				if(i != j) {
					double distance = findDistance(g.vertices.get(i), g.vertices.get(j));
					if(!canPass(distance)) {
						g.addEdgeToMatrix(i, j); // add edge to adjacency matrix
					}
				}
			}
		}
	}
	
	/**
	 * Checks to see if the graph is connected or not
	 * 
	 * @return connected - true is the graph is connected, false otherwise
	 */
	public boolean isConnected() {
		boolean connected = false;
		for(int j = 0; j<visited.length;j++) {
			if(visited[j] ==1) {
				connected = true;
			}else {
				connected = false;
			}
		}
		return connected;
	}

	/**
	 * Checks to see if the bear can pass
	 * 
	 * @param distance - distance between two vertices
	 * @return - true or false
	 */
	public boolean canPass(double distance) {
		return distance >= diameter ? true : false;
	}

	/**
	 * Calculates the distance between two vertices
	 * 
	 * @param a - vertex A
	 * @param b - vertex B
	 * @return distance - distance between two points
	 */
	public double findDistance(Vertex a, Vertex b) {
		double distance;
		distance = Math.sqrt((Math.pow( (double)(b.x - a.x), 2) 
				+ Math.pow((double)(b.y - a.y),2)));

		return distance;

	}

	/**
	 * Graph class
	 */
	public class Graph{
		public ArrayList<Vertex> vertices;
		public int V;
		public int[][] adjacencyMatrix;
		
		
		/**
		 * Creates a graph
		 * @param v - a vertex
		 */
		public Graph(int v) {
			vertices = new ArrayList<Vertex>(); // arraylist to store the vertices
			adjacencyMatrix = new int[v][v]; // 2d matrix to store the edges
			V = v;
		}

		public void addVertex(Vertex v) {
			vertices.add(v);
		}

		public void addEdgeToMatrix(int source, int dest) {
			adjacencyMatrix[source][dest] = 1;
		}

		public void printVertices() {
			for(int i = 0; i<vertices.size();i++){
				System.out.println(vertices.get(i));
			}
		}

		/**
		 * Prints the adjacency matrix
		 */
		public void printAdjacencyMatrix() {
			System.out.print("   ");
			for (int k = 0; k < V; k++) {
				System.out.print( " " + k );
			}

			System.out.print("\n   ");
			for (int l = 0; l < V; l++) {
				System.out.print("--");
			}
			System.out.println("");

			for(int i = 0; i < V; i++){
				System.out.print( i + " | ");
				for(int j = 0; j < V; j++) {
					System.out.print(adjacencyMatrix[i][j] + " ");
				}
				System.out.println();
			}
		}

	} // Graph

	/**
	 * Vertex class
	 */
	private class Vertex{
		public int x;
		public int y;

		/**
		 * Creates a Vertex object
		 * 
		 * @param myX - x coordinate
		 * @param myY - y coordinate
		 */
		public Vertex (int myX, int myY) {
			x = myX;
			y = myY;
		}

		@Override
		public String toString() {
			return ("(" + this.x + " , " + this.y + ")");
		}

	} // vertex

}
