
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

	public static void main(String args[]) {

		Vertex northWall;
		Vertex southWall;
		int diameter;

		RootBear rb = new RootBear();
		Graph graph = rb.new Graph();

		int[] firstLine = new int[4];

		Scanner sc = new Scanner(System.in);
		String line = sc.nextLine();
		Scanner lineScanner = new Scanner(line);

		for(int i = 0; i< 4; i++) {
			firstLine[i] = lineScanner.nextInt();
		}

		northWall = rb.new Vertex(firstLine[0]);
		southWall = rb.new Vertex(firstLine[1]);
		diameter = firstLine[2];

		line = sc.nextLine();
		Scanner vertexScanner = new Scanner(line);
		for(int i = 0; i < firstLine[3]; i++) {
			int x = vertexScanner.nextInt();
			int y = vertexScanner.nextInt();
			Vertex vertex = rb.new Vertex(x,y);
			graph.addVertex(vertex);
			line = sc.nextLine();
			vertexScanner = new Scanner(line);
		}

		vertexScanner.close();
		lineScanner.close();
		sc.close();

		//		graph.printVertices();

		rb.findAllEdges(graph, diameter, northWall, southWall);
		System.out.println("--------Vertices----------");
		graph.printVertices();
		System.out.println();
		System.out.println("--------Edges----------");
		graph.printEdges();

	} // main


	public void findAllEdges(Graph g, int diameter, Vertex northWall, Vertex southWall ) {
		for(int i = 0; i < g.vertices.size(); i++) {
			for(int j = 0; j<g.vertices.size(); j++) {
				if(i != j) {
					double distance = findDistance(g.vertices.get(i), g.vertices.get(j));
					if(!canPass(distance, diameter)) {
						g.addEdge(g.vertices.get(i), g.vertices.get(j));
					}
				}
			}
			
			double distanceNorth = g.vertices.get(i).y - northWall.y;
			double distanceSouth = southWall.y - g.vertices.get(i).y;
			
			if(!canPass(distanceNorth, diameter)) {
				g.addEdge(g.vertices.get(i),northWall);
			}
			
			if(!canPass(distanceSouth, diameter)) {
				g.addEdge(g.vertices.get(i),southWall);
			}
			
		}
		
	}

	public boolean canPass(double distance,int diameter) {
		return distance >= diameter ? true : false;
	}

	public double findDistance(Vertex a, Vertex b) {
		double distance;
		distance = Math.sqrt((Math.pow( (double)(b.x - a.x), 2) 
				+ Math.pow((double)(b.y - a.y),2)));

		return distance;

	} // distance


	public class Graph{
		public ArrayList<Vertex> vertices;
		public ArrayList<Edge> edges;

		public Graph() {
			vertices = new ArrayList<Vertex>();
			edges = new ArrayList<Edge>();
		}

		public void addVertex(Vertex v) {
			vertices.add(v);
		}

		public void addEdge(Vertex v, Vertex w) {
			Edge edge = new Edge(v,w);
			edges.add(edge);
		}

		public void printVertices() {
			for(int i = 0; i<vertices.size();i++){
				System.out.println(vertices.get(i));
			}
		}

		public void printEdges() {
			for(int i = 0; i<edges.size();i++){
				System.out.println(edges.get(i));
			}
		}

	} // Graph

	private class Vertex{
		public int x;
		public int y;

		public Vertex (int myX, int myY) {
			x = myX;
			y = myY;
		}

		public Vertex(int myY) {
			y = myY;
			x=1000;
		}

		@Override
		public String toString() {
			if(this.x == 1000) {
				return "wall";
			}
			return ("(" + this.x + " , " + this.y + ")");
		}

	} // vertex

	private class Edge{
		public Vertex a;
		public Vertex b;

		public Edge(Vertex vertexA, Vertex vertexB) {
			a = vertexA;
			b = vertexB;
		}

		@Override
		public String toString() {
			return (this.a + " ---- " + this.b);
		}

	} // Edge

}
