using System;
using System.Collections.Generic;
using System.Diagnostics;

class Program
{
    record Edge(int From, int To, double Weight);

    // --- Беллман-Форд ---
    static (Dictionary<int, double> dist, Dictionary<int, int?> parent) BellmanFord(Dictionary<int, List<Edge>> graph, int source)
    {
        var vertices = new HashSet<int>(graph.Keys);
        foreach (var edges in graph.Values)
            foreach (var e in edges) vertices.Add(e.To);

        var dist = new Dictionary<int, double>();
        var parent = new Dictionary<int, int?>();

        foreach (var v in vertices)
        {
            dist[v] = double.PositiveInfinity;
            parent[v] = null;
        }

        dist[source] = 0;

        for (int i = 0; i < vertices.Count - 1; i++)
        {
            bool updated = false;
            foreach (var u in graph.Keys)
                foreach (var edge in graph[u])
                    if (dist[u] + edge.Weight < dist[edge.To])
                    {
                        dist[edge.To] = dist[u] + edge.Weight;
                        parent[edge.To] = u;
                        updated = true;
                    }

            if (!updated) break;
        }

        foreach (var u in graph.Keys)
            foreach (var edge in graph[u])
                if (dist[u] + edge.Weight < dist[edge.To])
                    throw new InvalidOperationException("Граф содержит отрицательный цикл");

        return (dist, parent);
    }

    // --- Дейкстра ---
    static (Dictionary<int, double> dist, Dictionary<int, int?> parent) Dijkstra(Dictionary<int, List<Edge>> graph, int source)
    {
        var vertices = new HashSet<int>(graph.Keys);
        foreach (var edges in graph.Values)
            foreach (var e in edges) vertices.Add(e.To);

        var dist = new Dictionary<int, double>();
        var parent = new Dictionary<int, int?>();

        foreach (var v in vertices)
        {
            dist[v] = double.PositiveInfinity;
            parent[v] = null;
        }

        dist[source] = 0;
        var pq = new PriorityQueue<int, double>();
        pq.Enqueue(source, 0);

        while (pq.Count > 0)
        {
            pq.TryDequeue(out int u, out double du);
            if (du > dist[u]) continue;
            if (!graph.ContainsKey(u)) continue;

            foreach (var edge in graph[u])
            {
                double alt = dist[u] + edge.Weight;
                if (alt < dist[edge.To])
                {
                    dist[edge.To] = alt;
                    parent[edge.To] = u;
                    pq.Enqueue(edge.To, alt);
                }
            }
        }

        return (dist, parent);
    }

    // Восстановление пути
    static List<int> ReconstructPath(Dictionary<int, int?> parent, int target)
    {
        var path = new List<int>();
        int? cur = target;

        while (cur.HasValue)
        {
            path.Add(cur.Value);
            cur = parent[cur.Value];
        }

        path.Reverse();
        return path;
    }

    // Генерация случайного графа
    static Dictionary<int, List<Edge>> GenerateRandomGraph(int n, double p = 0.15, double minW = 1.0, double maxW = 20.0, bool nonNegative = true)
    {
        var rnd = new Random();
        var g = new Dictionary<int, List<Edge>>();

        for (int u = 0; u < n; u++) g[u] = new List<Edge>();

        for (int u = 0; u < n; u++)
            for (int v = 0; v < n; v++)
                if (u != v && rnd.NextDouble() <= p)
                {
                    double w = rnd.NextDouble() * (maxW - minW) + minW;
                    if (!nonNegative && rnd.NextDouble() < 0.05) w = -w;
                    g[u].Add(new Edge(u, v, w));
                }

        return g;
    }

    // Замер времени
    static (long bfMs, long? dijMs) MeasurePerformance(Dictionary<int, List<Edge>> g, int source)
    {
        var sw = Stopwatch.StartNew();
        try { BellmanFord(g, source); } catch { }
        sw.Stop();
        long bfMs = sw.ElapsedMilliseconds;

        bool allNonNeg = true;
        foreach (var edges in g.Values)
            foreach (var e in edges)
                if (e.Weight < 0) { allNonNeg = false; break; }

        long? dijMs = null;
        if (allNonNeg)
        {
            sw.Restart();
            Dijkstra(g, source);
            sw.Stop();
            dijMs = sw.ElapsedMilliseconds;
        }

        return (bfMs, dijMs);
    }

    static void Main()
    {
        // Малый тест
        Console.WriteLine("Тест: малый граф (фиксированный)");
        var graph1 = new Dictionary<int, List<Edge>>
        {
            [0] = new List<Edge> { new Edge(0, 1, 5), new Edge(0, 2, 2) },
            [1] = new List<Edge> { new Edge(1, 2, 1), new Edge(1, 3, 2) },
            [2] = new List<Edge> { new Edge(2, 1, 3), new Edge(2, 3, 7) },
            [3] = new List<Edge> { new Edge(3, 4, 1) },
            [4] = new List<Edge>()
        };

        var (bfDist, bfPar) = BellmanFord(graph1, 0);
        var (djDist, djPar) = Dijkstra(graph1, 0);

        Console.WriteLine("\nРезультаты Беллмана-Форда:");
        foreach (var kv in bfDist)
            Console.WriteLine($"dist[{kv.Key}] = {kv.Value}");

        Console.WriteLine("\nРезультаты Дейкстры:");
        foreach (var kv in djDist)
            Console.WriteLine($"dist[{kv.Key}] = {kv.Value}");

        Console.WriteLine("\nПуть до вершины 4 (Беллман-Форд): " + string.Join("->", ReconstructPath(bfPar, 4)));
        Console.WriteLine("Путь до вершины 4 (Дейкстра): " + string.Join("->", ReconstructPath(djPar, 4)));

        // Производительность на большом графе
        Console.WriteLine("\nТест производительности на случайном графе (n=500, p=0.15)");
        var g = GenerateRandomGraph(500, 0.15, 1, 20, true);
        var (bfMs, dijMs) = MeasurePerformance(g, 0);

        Console.WriteLine($"Bellman-Ford: {bfMs} мс, Dijkstra: {(dijMs.HasValue ? dijMs + " мс" : "н/д")}");

        // Демонстрация пути до случайной вершины
        var rnd = new Random();
        int randomTarget = rnd.Next(1, 500);

        var (bfDist2, bfPar2) = BellmanFord(g, 0);
        var (djDist2, djPar2) = Dijkstra(g, 0);

        Console.WriteLine($"\nСлучайный путь до вершины {randomTarget} (Беллман-Форд): " + string.Join("->", ReconstructPath(bfPar2, randomTarget)));
        Console.WriteLine($"Случайный путь до вершины {randomTarget} (Дейкстра): " + string.Join("->", ReconstructPath(djPar2, randomTarget)));

        Console.WriteLine("\nПрограмма завершена");
    }
}

