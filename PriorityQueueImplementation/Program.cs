using DoThiSoSanhPQ;
using System;
using System.Diagnostics;
using System.Drawing;

namespace DoThiSoSanhPQ
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

            // Sinh 1 lần duy nhất
            int[,] edges = new int[edgeCount, 3];
            CustomRandom rand = new CustomRandom(n * 31 + edgeCount); // chỉ dùng 1 seed duy nhất
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

            // Chạy thử 10 lần để lấy trung bình thời gian/memory nếu muốn
            for (int run = 1; run <= 10; run++)
            {
                Console.WriteLine($"\n========== LAN CHAY {run} ==========");

                Console.WriteLine("\n--- KET QUA SO SANH 4 HANG DOI UU TIEN ---");

                RunTest(new BinaryHeap(), edges, edgeCount, n, "Binary Heap");
                RunTest(new LinkedListQueue(), edges, edgeCount, n, "Double Linked List Queue");
                RunTest(new ArrayQueue(), edges, edgeCount, n, "Array Queue");
                RunTest(new BucketQueue(100 * n), edges, edgeCount, n, "Bucket Queue");
            }
        }


        static void RunTest(IPriorityQueue pq, int[,] edges, int edgeCount, int size, string title)
        {
            Graph graph = new Graph(size, pq);
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

            long memory = pq.EstimateMemory();

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

    class Graph
    {
        private EdgeNode[][] adjList;
        private int[] counts;
        private int size;
        private IPriorityQueue pq;
        private const int MAX = 1000;

        public Graph(int size, IPriorityQueue pq)
        {
            this.size = size;
            this.pq = pq;
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
            pq.Clear();

            for (int i = 0; i < size; i++)
                dist[i] = double.MaxValue;
            dist[start] = 0;
            pq.Enqueue(new PathNode(start, 0));

            while (pq.Count > 0)
            {
                PathNode node = pq.Dequeue();
                if (node == null) break;

                int u = node.Vertex;
                if (visited[u]) continue;
                visited[u] = true;

                for (int i = 0; i < counts[u]; i++)
                {
                    int v = adjList[u][i].Vertex;
                    double w = adjList[u][i].Weight;

                    if (w < 0 || double.IsNaN(w) || double.IsInfinity(w)) continue;

                    if (!visited[v] && dist[u] != double.MaxValue && dist[u] + w < dist[v])
                    {
                        dist[v] = dist[u] + w;
                        pq.Enqueue(new PathNode(v, dist[v]));
                    }
                }
            }

            return dist[end];
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
        void Clear();
        long EstimateMemory();
    }

    public class BinaryHeap : IPriorityQueue
    {
        private PathNode[] heap = new PathNode[1000000];
        private int size = 0;

        public int Count => size;

        public void Clear() => size = 0;

        public void Enqueue(PathNode node)
        {
            heap[size] = node;
            int i = size++;
            while (i > 0)
            {
                int parent = (i - 1) / 2;
                if (heap[i].Distance >= heap[parent].Distance) break;
                (heap[i], heap[parent]) = (heap[parent], heap[i]);
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

        public long EstimateMemory() => heap.Length * 16;
    }

    public class ArrayQueue : IPriorityQueue
    {
        private PathNode[] data = new PathNode[1000000];
        private int count = 0;

        public int Count => count;
        public void Clear() => count = 0;

        public void Enqueue(PathNode node)
        {
            data[count++] = node;
        }

        public PathNode Dequeue()
        {
            if (count == 0) return null;
            int minIndex = 0;
            for (int i = 1; i < count; i++)
            {
                if (data[i].Distance < data[minIndex].Distance)
                    minIndex = i;
            }
            PathNode minNode = data[minIndex];
            for (int i = minIndex; i < count - 1; i++)
                data[i] = data[i + 1];
            count--;
            return minNode;
        }

        public long EstimateMemory() => data.Length * 16;
    }

    public class LinkedListQueue : IPriorityQueue
    {
        private class Node { public PathNode Data; public Node Next, Prev; public Node(PathNode data) => Data = data; }
        private Node head;
        private int count;

        public int Count => count;
        private int maxCount = 0;

        public void Enqueue(PathNode node)
        {
            Node newNode = new(node);
            if (head == null) { head = newNode; }
            else
            {
                Node current = head;
                while (current.Next != null) current = current.Next;
                current.Next = newNode;
                newNode.Prev = current;
            }
            count++;
            if (count > maxCount) maxCount = count;
        }

        public PathNode Dequeue()
        {
            if (head == null) return null;
            Node min = head, current = head;
            while (current != null)
            {
                if (current.Data.Distance < min.Data.Distance)
                    min = current;
                current = current.Next;
            }

            if (min.Prev != null) min.Prev.Next = min.Next;
            if (min.Next != null) min.Next.Prev = min.Prev;
            if (min == head) head = min.Next;
            count--;
            return min.Data;
        }

        public void Clear()
        {
            head = null;
            count = 0;
            maxCount = 0;
        }
        public long EstimateMemory() => maxCount * 48;
    }

    public class BucketQueue : IPriorityQueue
    {
        private const int MAX_SIZE = 100000;
        private LinkedList<int>[] buckets;
        private int[] distMap;
        private int currentIndex;
        private int count;

        public BucketQueue(int maxDistance)
        {
            buckets = new LinkedList<int>[maxDistance + 1];
            for (int i = 0; i <= maxDistance; i++)
                buckets[i] = new LinkedList<int>();

            distMap = new int[MAX_SIZE];
            for (int i = 0; i < MAX_SIZE; i++)
                distMap[i] = -1;

            currentIndex = 0;
            count = 0;
        }

        public void Enqueue(PathNode node)
        {
            if (node.Vertex >= MAX_SIZE) return;
            int dist = (int)node.Distance;
            distMap[node.Vertex] = dist;
            buckets[dist].AddLast(node.Vertex);
            count++;
        }

        public PathNode Dequeue()
        {
            while (currentIndex < buckets.Length)
            {
                if (buckets[currentIndex].Count > 0)
                {
                    int v = buckets[currentIndex].First.Value;
                    buckets[currentIndex].RemoveFirst();
                    count--;
                    return new PathNode(v, currentIndex);
                }
                currentIndex++;
            }
            return null;
        }

        public int Count => count;

        public void Clear()
        {
            for (int i = 0; i < buckets.Length; i++)
                buckets[i].Clear();
            for (int i = 0; i < distMap.Length; i++)
                distMap[i] = -1;
            currentIndex = 0;
            count = 0;
        }

        public long EstimateMemory()
        {
            long bucketArraySize = buckets.Length * 8;
            long bucketContentSize = 0;
            for (int i = 0; i < buckets.Length; i++)
                bucketContentSize += buckets[i].Count * (4 + 16);
            long distMapSize = distMap.Length * 4;
            return bucketArraySize + bucketContentSize + distMapSize;
        }
    }

}


