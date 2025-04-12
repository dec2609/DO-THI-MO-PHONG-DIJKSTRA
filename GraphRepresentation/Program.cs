using System;
using System.Diagnostics;
using System.Collections.Generic;

namespace SinhDoThi
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.Write("Nhap so dinh: ");
            int n = int.Parse(Console.ReadLine());

            Console.Write("Do thi thua (1) hay day (2)? ");
            int choice = int.Parse(Console.ReadLine());

            int runs = 10;

            long maxEdgesLong = (long)n * (n - 1) / 2;
            int maxEdges = (maxEdgesLong > int.MaxValue) ? int.MaxValue : (int)maxEdgesLong;
            int edgeCount = (choice == 1) ? n + 5 : Math.Min(9000, maxEdges * 3 / 4);

            // Tạo đồ thị 1 lần duy nhất
            int[,] edges = new int[edgeCount, 3];
            CustomRandom rand = new CustomRandom(n * 31 + edgeCount + 42); // seed cố định

            int[,] allEdges = new int[maxEdges, 2];
            int idx = 0;

            for (int u = 0; u < n; u++)
            {
                for (int v = u + 1; v < n; v++)
                {
                    allEdges[idx, 0] = u;
                    allEdges[idx, 1] = v;
                    idx++;
                }
            }

            // Shuffle cạnh
            for (int i = maxEdges - 1; i > 0; i--)
            {
                int j = rand.Next(i + 1);
                (allEdges[i, 0], allEdges[j, 0]) = (allEdges[j, 0], allEdges[i, 0]);
                (allEdges[i, 1], allEdges[j, 1]) = (allEdges[j, 1], allEdges[i, 1]);
            }

            // Chọn cạnh ngẫu nhiên và gán trọng số
            for (int i = 0; i < edgeCount; i++)
            {
                int u = allEdges[i, 0];
                int v = allEdges[i, 1];
                int w = rand.Next(1, 101);
                edges[i, 0] = u;
                edges[i, 1] = v;
                edges[i, 2] = w;
            }

            int maxDegree = n;

            for (int run = 1; run <= runs; run++)
            {
                Console.WriteLine($"\n====== LAN CHAY {run} ======");

                GraphList gList = new GraphList(n);
                GraphMatrix gMatrix = new GraphMatrix(n);
                GraphMatrixListManual gMatList = new GraphMatrixListManual(n, maxDegree);
                GraphListMatrixManual gListMat = new GraphListMatrixManual(n, maxDegree);

                for (int i = 0; i < edgeCount; i++)
                {
                    int u = edges[i, 0];
                    int v = edges[i, 1];
                    int w = edges[i, 2];
                    gList.AddEdge(u, v, w);
                    gMatrix.AddEdge(u, v, w);
                    gMatList.AddEdge(u, v, w);
                    gListMat.AddEdge(u, v, w);
                }

                Stopwatch sw1 = Stopwatch.StartNew();
                double resultList = gList.Dijkstra(0, n - 1);
                sw1.Stop();
                long memoryList = EstimateMemoryGraphList(n, edgeCount);

                Stopwatch sw2 = Stopwatch.StartNew();
                double resultMatrix = gMatrix.Dijkstra(0, n - 1);
                sw2.Stop();
                long memoryMatrix = EstimateMemoryGraphMatrix(n);

                Stopwatch sw3 = Stopwatch.StartNew();
                double resultMatList = gMatList.Dijkstra(0, n - 1);
                sw3.Stop();
                long memoryMatList = EstimateMemoryGraphMatrix(n) + EstimateMemoryGraphList(n, edgeCount);

                Stopwatch sw4 = Stopwatch.StartNew();
                double resultListMat = gListMat.Dijkstra(0, n - 1);
                sw4.Stop();
                long memoryListMat = EstimateMemoryGraphMatrix(n) + EstimateMemoryGraphList(n, edgeCount);

                Console.WriteLine($"- So dinh: {n}, So canh: {edgeCount}");

                Console.WriteLine($"\n[DANH SACH KE]");
                Console.WriteLine($"  Ket qua: {resultList:F2}");
                Console.WriteLine($"  Thoi gian: {sw1.Elapsed.TotalMilliseconds:F2} ms");
                Console.WriteLine($"  Bo nho uoc tinh: {memoryList / 1024.0:F2} KB");

                Console.WriteLine($"\n[MA TRAN KE]");
                Console.WriteLine($"  Ket qua: {resultMatrix:F2}");
                Console.WriteLine($"  Thoi gian: {sw2.Elapsed.TotalMilliseconds:F2} ms");
                Console.WriteLine($"  Bo nho uoc tinh: {memoryMatrix / 1024.0:F2} KB");

                Console.WriteLine($"\n[MA TRAN + DANH SACH]");
                Console.WriteLine($"  Ket qua: {resultMatList:F2}");
                Console.WriteLine($"  Thoi gian: {sw3.Elapsed.TotalMilliseconds:F2} ms");
                Console.WriteLine($"  Bo nho uoc tinh: {memoryMatList / 1024.0:F2} KB");

                Console.WriteLine($"\n[DANH SACH + MA TRAN]");
                Console.WriteLine($"  Ket qua: {resultListMat:F2}");
                Console.WriteLine($"  Thoi gian: {sw4.Elapsed.TotalMilliseconds:F2} ms");
                Console.WriteLine($"  Bo nho uoc tinh: {memoryListMat / 1024.0:F2} KB");
            }
        }




        // Tạo số ngẫu nhiên thủ công
        class CustomRandom
        {
            private int seed;
            public CustomRandom(int seed) { this.seed = seed; }

            public int Next(int max)
            {
                seed = (seed * 1103515245 + 12345) & int.MaxValue;
                return seed % max;
            }

            public int Next(int min, int max)
            {
                return min + Next(max - min);
            }
        }

        // Kiểm tra trùng cạnh thủ công
        static bool EdgeExists(int[,] edges, int count, int u, int v)
        {
            for (int i = 0; i < count; i++)
            {
                if ((edges[i, 0] == u && edges[i, 1] == v) || (edges[i, 0] == v && edges[i, 1] == u))
                    return true;
            }
            return false;
        }
        static long EstimateMemoryGraphList(int vertices, int edges)
        {
            // Danh sách kề: mỗi đỉnh có danh sách các cạnh
            // Mỗi cạnh lưu: vertex (int - 4 byte), weight (double - 8 byte), pointer (Next - 8 byte)
            long perEdgeSize = 4 + 8 + 8; // = 20 byte (gần đúng)
            long listNodeArray = vertices * 8; // Mỗi đỉnh là 1 pointer tới danh sách (8 byte)
            long edgeNodes = edges * 2 * perEdgeSize; // Mỗi cạnh lưu 2 lần (vô hướng)
            return listNodeArray + edgeNodes;
        }

        static long EstimateMemoryGraphMatrix(int vertices)
        {
            // Ma trận kề: lưu 1 ma trận 2 chiều double[V][V], mỗi double là 8 byte
            return (long)vertices * vertices * 8;
        }


        static void GenerateRandomGraph(int n, int edgeCount, GraphList gList, GraphMatrix gMatrix)
        {
            int added = 0;
            Random rand = new Random();
            bool[,] used = new bool[n, n];

            while (added < edgeCount)
            {
                int u = rand.Next(n);
                int v = rand.Next(n);
                if (u != v && !used[u, v])
                {
                    double w = rand.Next(1, 101); // Trọng số từ 1 → 100
                    if (gList != null) gList.AddEdge(u, v, w);
                    if (gMatrix != null) gMatrix.AddEdge(u, v, w);
                    used[u, v] = true;
                    used[v, u] = true;
                    added++;
                }
            }
        }

    }

    class Node
    {
        public int Vertex;
        public int Weight;
        public Node Next;

        public Node(int vertex, int weight)
        {
            Vertex = vertex;
            Weight = weight;
            Next = null;
        }
    }

    class LinkedList
    {
        public Node Head;

        public void Add(int vertex, int weight)
        {
            Node newNode = new Node(vertex, weight);
            newNode.Next = Head;
            Head = newNode;
        }

        public void Print(int from)
        {
            Node current = Head;
            while (current != null)
            {
                Console.Write($"({from} -> {current.Vertex}, w={current.Weight}) ");
                current = current.Next;
            }
        }
    }

    class Graph
    {
        private LinkedList[] adjacencyList;
        private int vertexCount;
        private bool[,] addedEdges;
        private Random rand;

        public Graph(int vertexCount)
        {
            this.vertexCount = vertexCount;
            adjacencyList = new LinkedList[vertexCount];
            for (int i = 0; i < vertexCount; i++)
                adjacencyList[i] = new LinkedList();

            addedEdges = new bool[vertexCount, vertexCount];
            rand = new Random();
        }

        public void AddEdge(int u, int v, int weight)
        {
            if (!addedEdges[u, v])
            {
                adjacencyList[u].Add(v, weight);
                adjacencyList[v].Add(u, weight); // Vô hướng
                addedEdges[u, v] = true;
                addedEdges[v, u] = true;
            }
        }

        public void GenerateRandomEdges(int edgeCount)
        {
            int added = 0;
            while (added < edgeCount)
            {
                int u = rand.Next(vertexCount);
                int v = rand.Next(vertexCount);
                if (u != v && !addedEdges[u, v])
                {
                    int weight = rand.Next(1, 101); // Trọng số 1 → 100
                    AddEdge(u, v, weight);
                    added++;
                }
            }
        }
    }


    public class PathNode
    {
        public int Vertex;
        public double Distance;

        public PathNode(int vertex, double distance)
        {
            Vertex = vertex;
            Distance = distance;
        }
    }

    public interface IPriorityQueue
    {
        void Enqueue(PathNode node);
        PathNode Dequeue();
        int Count { get; }
    }

    public class BinaryHeap : IPriorityQueue
    {
        private PathNode[] heap = new PathNode[100000000];
        private int size = 0;

        public int Count => size;

        public void Enqueue(PathNode node)
        {
            if (size == heap.Length)
                ExpandHeap();

            heap[size] = node;
            int i = size++;
            while (i > 0)
            {
                int parent = (i - 1) / 2;
                if (heap[parent].Distance <= heap[i].Distance) break;
                (heap[parent], heap[i]) = (heap[i], heap[parent]);
                i = parent;
            }
        }

        public PathNode Dequeue()
        {
            if (size == 0) return null;
            PathNode result = heap[0];
            heap[0] = heap[--size];

            int i = 0;
            while (true)
            {
                int left = 2 * i + 1;
                int right = 2 * i + 2;
                int smallest = i;

                if (left < size && heap[left].Distance < heap[smallest].Distance)
                    smallest = left;

                if (right < size && heap[right].Distance < heap[smallest].Distance)
                    smallest = right;

                if (smallest == i) break;

                (heap[i], heap[smallest]) = (heap[smallest], heap[i]);
                i = smallest;
            }

            return result;
        }

        private void ExpandHeap()
        {
            int newSize = heap.Length * 2;
            PathNode[] newHeap = new PathNode[newSize];
            for (int i = 0; i < heap.Length; i++)
                newHeap[i] = heap[i];
            heap = newHeap;
        }
    }


    public class EdgeNode
    {
        public int Vertex;
        public double Weight;
        public EdgeNode Next;

        public EdgeNode(int vertex, double weight)
        {
            Vertex = vertex;
            Weight = weight;
            Next = null;
        }
    }


    public class GraphMatrix
    {
        private double[,] matrix;
        private int size;

        public GraphMatrix(int size)
        {
            this.size = size;
            matrix = new double[size, size];
            for (int i = 0; i < size; i++)
                for (int j = 0; j < size; j++)
                    matrix[i, j] = (i == j) ? 0 : double.MaxValue;
        }

        public void AddEdge(int u, int v, double weight)
        {
            matrix[u, v] = weight;
            matrix[v, u] = weight;
        }

        public double Dijkstra(int start, int end)
        {
            double[] dist = new double[size];
            bool[] visited = new bool[size];

            for (int i = 0; i < size; i++)
                dist[i] = double.MaxValue;
            dist[start] = 0;

            IPriorityQueue pq = new BinaryHeap();
            pq.Enqueue(new PathNode(start, 0));

            while (pq.Count > 0)
            {
                PathNode node = pq.Dequeue();
                int u = node.Vertex;
                if (visited[u]) continue;
                if (dist[u] == double.MaxValue) continue;


                for (int v = 0; v < size; v++)
                {
                    if (!visited[v] && matrix[u, v] < double.MaxValue && dist[u] < double.MaxValue)
{
    double alt = dist[u] + matrix[u, v];
    if (alt < dist[v])
    {
        dist[v] = alt;
        pq.Enqueue(new PathNode(v, alt));
    }
}

                }
            }

            return dist[end];
        }
    }
    public class GraphList
    {
        private EdgeNode[] adjList;
        private int size;

        public GraphList(int size)
        {
            this.size = size;
            adjList = new EdgeNode[size];
        }

        public void AddEdge(int u, int v, double weight)
        {
            EdgeNode newNode = new EdgeNode(v, weight);
            newNode.Next = adjList[u];
            adjList[u] = newNode;

            EdgeNode newNode2 = new EdgeNode(u, weight);
            newNode2.Next = adjList[v];
            adjList[v] = newNode2;
        }

        public double Dijkstra(int start, int end)
        {
            double[] dist = new double[size];
            bool[] visited = new bool[size];
            IPriorityQueue pq = new BinaryHeap();

            for (int i = 0; i < size; i++)
                dist[i] = double.MaxValue;

            dist[start] = 0;
            pq.Enqueue(new PathNode(start, 0));

            while (pq.Count > 0)
            {
                PathNode node = pq.Dequeue();
                int u = node.Vertex;
                if (visited[u]) continue;
                if (dist[u] == double.MaxValue) continue;
                visited[u] = true;

                EdgeNode current = adjList[u];
                while (current != null)
                {
                    int v = current.Vertex;
                    double weight = current.Weight;

                    if (!visited[v] && dist[u] < double.MaxValue)
                    {
                        double alt = dist[u] + weight;
                        if (alt < dist[v])
                        {
                            dist[v] = alt;
                            pq.Enqueue(new PathNode(v, alt));
                        }
                    }

                    current = current.Next;
                }

            }

            return dist[end];
        }
    }

    public class GraphMatrixListManual
    {
        private double[,] matrix;
        private int[][] neighbors;
        private int[] neighborCounts;
        private int size;

        public GraphMatrixListManual(int size, int maxDegree)
        {
            this.size = size;
            matrix = new double[size, size];
            neighbors = new int[size][];
            neighborCounts = new int[size];

            for (int i = 0; i < size; i++)
            {
                neighbors[i] = new int[maxDegree];
                neighborCounts[i] = 0;
                for (int j = 0; j < size; j++)
                    matrix[i, j] = (i == j) ? 0 : double.MaxValue;
            }
        }

        public void AddEdge(int u, int v, double weight)
        {
            matrix[u, v] = weight;
            matrix[v, u] = weight;

            neighbors[u][neighborCounts[u]++] = v;
            neighbors[v][neighborCounts[v]++] = u;
        }

        public double Dijkstra(int start, int end)
        {
            double[] dist = new double[size];
            bool[] visited = new bool[size];
            IPriorityQueue pq = new BinaryHeap();

            for (int i = 0; i < size; i++)
                dist[i] = double.MaxValue;
            dist[start] = 0;
            pq.Enqueue(new PathNode(start, 0));

            while (pq.Count > 0)
            {
                PathNode node = pq.Dequeue();
                int u = node.Vertex;
                if (visited[u] || dist[u] == double.MaxValue) continue;
                visited[u] = true;

                for (int i = 0; i < neighborCounts[u]; i++)
                {
                    int v = neighbors[u][i];
                    double weight = matrix[u, v];

                    if (!visited[v] && dist[u] < double.MaxValue)
                    {
                        double alt = dist[u] + weight;
                        if (alt < dist[v])
                        {
                            dist[v] = alt;
                            pq.Enqueue(new PathNode(v, alt));
                        }
                    }
                }
            }

            return dist[end];
        }

    }

    public class GraphListMatrixManual
    {
        private int[][] adjList;
        private double[,] matrix;
        private int[] neighborCounts;
        private int size;

        public GraphListMatrixManual(int size, int maxDegree)
        {
            this.size = size;
            matrix = new double[size, size];
            adjList = new int[size][];
            neighborCounts = new int[size];

            for (int i = 0; i < size; i++)
            {
                adjList[i] = new int[maxDegree];
                neighborCounts[i] = 0;
                for (int j = 0; j < size; j++)
                    matrix[i, j] = (i == j) ? 0 : double.MaxValue;
            }
        }

        public void AddEdge(int u, int v, double weight)
        {
            matrix[u, v] = weight;
            matrix[v, u] = weight;

            adjList[u][neighborCounts[u]++] = v;
            adjList[v][neighborCounts[v]++] = u;
        }

        public double Dijkstra(int start, int end)
        {
            double[] dist = new double[size];
            bool[] visited = new bool[size];
            IPriorityQueue pq = new BinaryHeap();

            for (int i = 0; i < size; i++)
                dist[i] = double.MaxValue;

            dist[start] = 0;
            pq.Enqueue(new PathNode(start, 0));

            while (pq.Count > 0)
            {
                PathNode node = pq.Dequeue();
                int u = node.Vertex;

                if (visited[u] || dist[u] == double.MaxValue) continue;
                visited[u] = true;

                for (int i = 0; i < neighborCounts[u]; i++)
                {
                    int v = adjList[u][i];
                    double weight = matrix[u, v];

                    if (!visited[v])
                    {
                        double alt = dist[u] + weight;
                        if (alt < dist[v])
                        {
                            dist[v] = alt;
                            pq.Enqueue(new PathNode(v, alt));
                        }
                    }
                }
            }

            return dist[end];
        }

    }


}




