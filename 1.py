# =============================================================================
# Program 1: Dijkstra's Algorithm - Indian Cities Road Network
# Finds shortest path from a source city to all other cities.
# =============================================================================

import heapq

# ── City list ──────────────────────────────────────────────────────────────
CITIES = [
    "Delhi",        #  0
    "Jaipur",       #  1
    "Agra",         #  2
    "Chandigarh",   #  3
    "Lucknow",      #  4
    "Kanpur",       #  5
    "Varanasi",     #  6
    "Patna",        #  7
    "Kolkata",      #  8
    "Bhubaneswar",  #  9
    "Mumbai",       # 10
    "Pune",         # 11
    "Ahmedabad",    # 12
    "Nagpur",       # 13
    "Hyderabad",    # 14
    "Chennai",      # 15
    "Bangalore",    # 16
    "Kochi",        # 17
    "Bhopal",       # 18
    "Indore",       # 19
    "Surat",        # 20
    "Amritsar",     # 21
    "Shimla",       # 22
    "Dehradun",     # 23
    "Guwahati",     # 24
]

N = len(CITIES)

# ── Build road network (adjacency list) ────────────────────────────────────
def build_graph():
    # graph[u] = list of (v, distance_km)
    graph = {i: [] for i in range(N)}

    edges = [
        # Delhi (0)
        (0,  1,  281), (0,  2,  233), (0,  3,  243),
        (0,  4,  555), (0, 12,  940), (0, 21,  449), (0, 23,  295),
        # Jaipur (1)
        (1,  2,  232), (1, 12,  656), (1, 19,  490),
        # Agra (2)
        (2,  4,  363), (2, 13,  765),
        # Chandigarh (3)
        (3, 21,  229), (3, 22,  115),
        # Lucknow (4)
        (4,  5,   83), (4,  6,  286), (4, 18,  493),
        # Varanasi (6)
        (6,  7,  281),
        # Patna (7)
        (7,  8,  581), (7, 24, 1310),
        # Kolkata (8)
        (8,  9,  441), (8, 24, 1030),
        # Bhubaneswar (9)
        (9, 13, 1026), (9, 15, 1043),
        # Mumbai (10)
        (10, 11,  148), (10, 12,  524), (10, 13,  831),
        (10, 16,  984), (10, 20,  280),
        # Pune (11)
        (11, 14,  563), (11, 16,  839),
        # Ahmedabad (12)
        (12, 19,  545), (12, 20,  262),
        # Nagpur (13)
        (13, 14,  500), (13, 18,  357),
        # Hyderabad (14)
        (14, 15,  626), (14, 16,  568),
        # Chennai (15)
        (15, 16,  346), (15, 17,  682),
        # Bangalore (16)
        (16, 17,  549),
        # Bhopal (18)
        (18, 19,  192),
        # Dehradun (23)
        (23,  3,  174), (23, 22,  235),
    ]

    for u, v, d in edges:
        graph[u].append((v, d))
        graph[v].append((u, d))     # undirected

    return graph

# ── Dijkstra ────────────────────────────────────────────────────────────────
def dijkstra(graph, src):
    INF    = float('inf')
    dist   = [INF] * N
    parent = [-1]  * N
    dist[src] = 0

    # min-heap: (cost, node)
    pq = [(0, src)]

    while pq:
        cost, u = heapq.heappop(pq)
        if cost > dist[u]:          # stale entry
            continue
        for v, w in graph[u]:
            if dist[u] + w < dist[v]:
                dist[v]   = dist[u] + w
                parent[v] = u
                heapq.heappush(pq, (dist[v], v))

    return dist, parent

# ── Reconstruct path ────────────────────────────────────────────────────────
def get_path(parent, src, dst):
    path = []
    cur  = dst
    while cur != -1:
        path.append(CITIES[cur])
        cur = parent[cur]
    path.reverse()
    return " -> ".join(path)

# ── Print results ────────────────────────────────────────────────────────────
def print_results(src, dist, parent):
    print()
    print("=" * 65)
    print(f"  Dijkstra's Shortest Paths from: {CITIES[src]}")
    print("=" * 65)
    print(f"{'No.':<5} {'Destination':<20} {'Distance(km)':<15} Route")
    print(f"{'---':<5} {'-----------':<20} {'------------':<15} -----")

    sno = 1
    for i in range(N):
        if i == src:
            continue
        dest = CITIES[i]
        if dist[i] == float('inf'):
            print(f"{sno:<5} {dest:<20} {'Unreachable':<15} No path")
        else:
            route = get_path(parent, src, i)
            print(f"{sno:<5} {dest:<20} {dist[i]:<15} {route}")
        sno += 1
    print("=" * 65)

# ── Main ────────────────────────────────────────────────────────────────────
def main():
    print("*" * 53)
    print("*  Dijkstra's Algorithm - Indian Cities Road Network *")
    print("*" * 53)
    print()
    print("Available Cities:")
    print("-" * 40)
    for i, city in enumerate(CITIES):
        print(f"  {i:>2}. {city}")
    print("-" * 40)

    while True:
        try:
            src = int(input(f"\nEnter source city number (0 to {N-1}): "))
            if 0 <= src < N:
                break
            print(f"  Please enter a number between 0 and {N-1}.")
        except ValueError:
            print("  Invalid input. Please enter a number.")

    graph        = build_graph()
    dist, parent = dijkstra(graph, src)
    print_results(src, dist, parent)

if __name__ == "__main__":
    main()