
//import statements
import java.util.*;

/**
 * Root bear Algorithm
 * 
 * @author Akanksha
 * @since December 02, 2017
 *
 */
public class RootBear {

	private static final int SOURCE = 0;
	private Queue<Integer> queue;
	private int[] visited;

	public static void main(String args[]) {

		Vertex northWall;
		Vertex southWall;
		int diameter;
		int trees;
		RootBear rb = new RootBear();
		Graph graph;
		int[] firstLine = new int[4];

		Scanner sc = new Scanner(System.in);
		String line = sc.nextLine();
		Scanner lineScanner = new Scanner(line);

		for(int i = 0; i< 4; i++) {
			firstLine[i] = lineScanner.nextInt();
		}

		diameter = firstLine[2];
		trees = firstLine[3];
		graph = rb.new Graph(trees + 2);

		northWall = rb.new Vertex(0,firstLine[0]);
		southWall = rb.new Vertex(0, firstLine[1]);		
		graph.addVertex(northWall);
		graph.addVertex(southWall);

		Scanner vertexScanner;
		
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
		System.out.println("--------------------------");

		lineScanner.close();
		sc.close();

		rb.findAllEdges(graph, diameter, northWall, southWall);

		rb.bfs(graph.adjacencyMatrix,SOURCE);
		
		if(rb.isConnected()) {
			System.out.println("Can pass?: NO");
		}else {
			System.out.println("Can pass?: YES");
		}

	} // main

	/**
	 * 
	 * @param adj_matrix
	 * @param source
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
	 * 
	 * @param g
	 * @param diameter
	 * @param northWall
	 * @param southWall
	 */
	public void findAllEdges(Graph g, int diameter, Vertex northWall, Vertex southWall ) {
		for(int i = 0; i < g.vertices.size(); i++) {
			for(int j = 0; j<g.vertices.size(); j++) {
				if(i != j) {
					double distance = findDistance(g.vertices.get(i), g.vertices.get(j));
					if(!canPass(distance, diameter)) {
						g.addEdgeToMatrix(i, j);
					}
				}
			}
		}
	}
	
	/**
	 * 
	 * @return
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
	 * 
	 * @param distance
	 * @param diameter
	 * @return
	 */
	public boolean canPass(double distance,int diameter) {
		return distance >= diameter ? true : false;
	}

	/**
	 * 
	 * @param a
	 * @param b
	 * @return
	 */
	public double findDistance(Vertex a, Vertex b) {
		double distance;
		distance = Math.sqrt((Math.pow( (double)(b.x - a.x), 2) 
				+ Math.pow((double)(b.y - a.y),2)));

		return distance;

	}

	/**
	 * 
	 *
	 *
	 */
	public class Graph{
		public ArrayList<Vertex> vertices;
		public int V;
		public int[][] adjacencyMatrix;
		
		
		/**
		 * 
		 * @param v
		 */
		public Graph(int v) {
			vertices = new ArrayList<Vertex>();
			adjacencyMatrix = new int[v][v];
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
	 * 
	 * @author Akanksha
	 *
	 */
	private class Vertex{
		public int x;
		public int y;

		/**
		 * Vertex 
		 * 
		 * @param myX
		 * @param myY
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
