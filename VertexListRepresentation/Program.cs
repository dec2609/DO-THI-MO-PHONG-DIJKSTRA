using System;
using System.Diagnostics;

namespace DoThiSoSanh
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

            int[,] edges = new int[edgeCount, 3];
            CustomRandom rand = new CustomRandom(n * 31 + edgeCount); // Giữ nguyên seed

            int added = 0;
            while (added < edgeCount)
            {
                int u = rand.Next(n);
                int v = rand.Next(n);
                if (u != v && !EdgeExists(edges, added, u, v))
                {
                    int w = rand.Next(1, 101);
                    edges[added, 0] = u;
                    edges[added, 1] = v;
                    edges[added, 2] = w;
                    added++;
                }
            }

            for (int round = 1; round <= 10; round++)
            {
                Console.WriteLine($"\n========== LAN CHAY {round} ==========");

                RunTest(new GraphArray(n), edges, edgeCount, n, "Mang Co Dinh");
                RunTest(new GraphCustomArrayList(n), edges, edgeCount, n, "ArrayList Tu Cai");
                RunTest(new GraphLinkedList(n), edges, edgeCount, n, "LinkedList Tu Cai");
            }
        }



        static void RunTest(IGraph graph, int[,] edges, int edgeCount, int size, string title)
        {
            for (int i = 0; i < edgeCount; i++)
            {
                int u = edges[i, 0];
                int v = edges[i, 1];
                int w = edges[i, 2];
                graph.AddEdge(u, v, w);
            }

            Stopwatch sw = Stopwatch.StartNew();
            double result = graph.Dijkstra(0, size - 1);
            sw.Stop();

            long memory = graph.EstimateMemory();

            Console.WriteLine($"\n=== {title} ===");
            Console.WriteLine($"Ket qua: {result:F2}");
            Console.WriteLine($"Thoi gian: {sw.Elapsed.TotalMilliseconds:F2} ms");
            Console.WriteLine($"Bo nho uoc tinh: {memory / 1024.0:F2} KB");
        }

        static bool EdgeExists(int[,] edges, int count, int u, int v)
        {
            for (int i = 0; i < count; i++)
            {
                if ((edges[i, 0] == u && edges[i, 1] == v) || (edges[i, 0] == v && edges[i, 1] == u))
                    return true;
            }
            return false;
        }
    }

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

    public interface IGraph
    {
        void AddEdge(int u, int v, double w);
        double Dijkstra(int start, int end);
        long EstimateMemory();
    }

    class GraphArray : IGraph
    {
        private EdgeNode[][] adjList;
        private int[] counts;
        private int size;
        private const int MAX = 1000;

        public GraphArray(int size)
        {
            this.size = size;
            adjList = new EdgeNode[size][];
            counts = new int[size];
            for (int i = 0; i < size; i++)
                adjList[i] = new EdgeNode[MAX];
        }

        public void AddEdge(int u, int v, double w)
        {
            adjList[u][counts[u]++] = new EdgeNode(v, w);
            adjList[v][counts[v]++] = new EdgeNode(u, w);
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
                visited[u] = true;

                for (int i = 0; i < counts[u]; i++)
                {
                    int v = adjList[u][i].Vertex;
                    double w = adjList[u][i].Weight;

                    // FIX TRÀN SỐ
                    if (!visited[v] && dist[u] < double.MaxValue && dist[u] + w < dist[v])
                    {
                        dist[v] = dist[u] + w;
                        pq.Enqueue(new PathNode(v, dist[v]));
                    }
                }
            }

            return dist[end];
        }


        public long EstimateMemory()
        {
            return size * MAX * (4 + 8);
        }
    }

    class GraphCustomArrayList : IGraph
    {
        private EdgeNode[][] adjList;
        private int[] capacities;
        private int[] counts;
        private int size;

        public GraphCustomArrayList(int size)
        {
            this.size = size;
            adjList = new EdgeNode[size][];
            capacities = new int[size];
            counts = new int[size];
            for (int i = 0; i < size; i++)
            {
                capacities[i] = 4;
                adjList[i] = new EdgeNode[capacities[i]];
            }
        }

        public void AddEdge(int u, int v, double w)
        {
            Add(u, v, w);
            Add(v, u, w);
        }

        private void Add(int u, int v, double w)
        {
            if (counts[u] == capacities[u])
            {
                capacities[u] *= 2;
                EdgeNode[] newArray = new EdgeNode[capacities[u]];
                for (int i = 0; i < counts[u]; i++)
                    newArray[i] = adjList[u][i];
                adjList[u] = newArray;
            }
            adjList[u][counts[u]++] = new EdgeNode(v, w);
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
                visited[u] = true;

                for (int i = 0; i < counts[u]; i++)
                {
                    int v = adjList[u][i].Vertex;
                    double w = adjList[u][i].Weight;

                    // ✅ Fix tràn số ở đây
                    if (!visited[v] && dist[u] < double.MaxValue && dist[u] + w < dist[v])
                    {
                        dist[v] = dist[u] + w;
                        pq.Enqueue(new PathNode(v, dist[v]));
                    }
                }
            }

            return dist[end];
        }


        public long EstimateMemory()
        {
            long total = 0;
            for (int i = 0; i < size; i++)
                total += capacities[i] * (4 + 8);
            return total;
        }
    }

    class GraphLinkedList : IGraph
    {
        private LinkedEdge[] heads;
        private int size;

        public GraphLinkedList(int size)
        {
            this.size = size;
            heads = new LinkedEdge[size];
        }

        public void AddEdge(int u, int v, double w)
        {
            heads[u] = new LinkedEdge(v, w, heads[u]);
            heads[v] = new LinkedEdge(u, w, heads[v]);
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
                visited[u] = true;

                for (LinkedEdge edge = heads[u]; edge != null; edge = edge.Next)
                {
                    int v = edge.Vertex;
                    double w = edge.Weight;

                    // ✅ Fix tràn số ở đây
                    if (!visited[v] && dist[u] != double.MaxValue && dist[u] + w < dist[v])
                    {
                        dist[v] = dist[u] + w;
                        pq.Enqueue(new PathNode(v, dist[v]));
                    }
                }
            }

            return dist[end];
        }


        public long EstimateMemory()
        {
            // Ước lượng mỗi đỉnh có 10 cạnh
            return size * 10 * (4 + 8 + 8);
        }
    }

    public class EdgeNode
    {
        public int Vertex;
        public double Weight;
        public EdgeNode(int vertex, double weight)
        {
            Vertex = vertex;
            Weight = weight;
        }
    }

    public class LinkedEdge
    {
        public int Vertex;
        public double Weight;
        public LinkedEdge Next;
        public LinkedEdge(int vertex, double weight, LinkedEdge next)
        {
            Vertex = vertex;
            Weight = weight;
            Next = next;
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
        private PathNode[] heap = new PathNode[1000000];
        private int size = 0;
        public int Count => size;

        public void Enqueue(PathNode node)
        {
            if (size == heap.Length) Expand();
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
            PathNode root = heap[0];
            heap[0] = heap[--size];
            int i = 0;
            while (true)
            {
                int left = 2 * i + 1, right = 2 * i + 2, smallest = i;
                if (left < size && heap[left].Distance < heap[smallest].Distance) smallest = left;
                if (right < size && heap[right].Distance < heap[smallest].Distance) smallest = right;
                if (smallest == i) break;
                (heap[i], heap[smallest]) = (heap[smallest], heap[i]);
                i = smallest;
            }
            return root;
        }

        private void Expand()
        {
            var newHeap = new PathNode[heap.Length * 2];
            for (int i = 0; i < heap.Length; i++) newHeap[i] = heap[i];
            heap = newHeap;
        }
    }
}
