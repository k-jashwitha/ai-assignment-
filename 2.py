3
# =============================================================================
# Program 2: UGV Pathfinding - Static Obstacles (70x70 Grid)
# Algorithm : A* Search with 8-directional movement
# Obstacles : Randomly generated at 3 density levels
# Output    : Optimal path + Measures of Effectiveness (MOE)
# =============================================================================

import heapq
import random
import time

# ── Constants ──────────────────────────────────────────────────────────────
ROWS = 70
COLS = 70

# Cell types
FREE      = 0
OBSTACLE  = 1
PATH      = 2
START     = 3
GOAL      = 4

# Movement costs (scaled x10 to match C version)
STRAIGHT = 10
DIAGONAL = 14

# 8-directional moves: (row_offset, col_offset, cost)
DIRECTIONS = [
    (-1, -1, DIAGONAL), (-1,  0, STRAIGHT), (-1,  1, DIAGONAL),
    ( 0, -1, STRAIGHT),                     ( 0,  1, STRAIGHT),
    ( 1, -1, DIAGONAL), ( 1,  0, STRAIGHT), ( 1,  1, DIAGONAL),
]

DENSITY_LEVELS = {
    1: ("Low",    0.10),
    2: ("Medium", 0.25),
    3: ("High",   0.40),
}

# ── Heuristic: Octile distance ──────────────────────────────────────────────
def heuristic(r1, c1, r2, c2):
    dr = abs(r1 - r2)
    dc = abs(c1 - c2)
    mn = min(dr, dc)
    mx = max(dr, dc)
    return DIAGONAL * mn + STRAIGHT * (mx - mn)

# ── Obstacle generation ─────────────────────────────────────────────────────
def generate_obstacles(grid, density, sr, sc, gr, gc):
    total   = int(ROWS * COLS * density)
    placed  = 0
    while placed < total:
        r = random.randint(0, ROWS - 1)
        c = random.randint(0, COLS - 1)
        if grid[r][c] == FREE and not (r == sr and c == sc) \
                               and not (r == gr and c == gc):
            grid[r][c] = OBSTACLE
            placed += 1

# ── A* Search ───────────────────────────────────────────────────────────────
def astar(grid, sr, sc, gr, gc):
    INF    = float('inf')
    g_cost = [[INF] * COLS for _ in range(ROWS)]
    parent = [[None]  * COLS for _ in range(ROWS)]
    closed = [[False] * COLS for _ in range(ROWS)]

    g_cost[sr][sc] = 0
    h               = heuristic(sr, sc, gr, gc)
    # heap entries: (f, g, row, col)
    heap = [(h, 0, sr, sc)]
    nodes_explored = 0

    while heap:
        f, g, r, c = heapq.heappop(heap)

        if closed[r][c]:
            continue
        closed[r][c] = True
        nodes_explored += 1

        # Goal reached
        if r == gr and c == gc:
            # Trace path
            path = []
            cur  = (gr, gc)
            while cur is not None:
                path.append(cur)
                cur = parent[cur[0]][cur[1]]
            path.reverse()
            return path, g_cost[gr][gc], nodes_explored

        for dr, dc, cost in DIRECTIONS:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < ROWS and 0 <= nc < COLS):
                continue
            if grid[nr][nc] == OBSTACLE:
                continue
            if closed[nr][nc]:
                continue
            # Prevent diagonal corner-cutting
            if dr != 0 and dc != 0:
                if grid[r + dr][c] == OBSTACLE and \
                   grid[r][c + dc] == OBSTACLE:
                    continue

            ng = g + cost
            if ng < g_cost[nr][nc]:
                g_cost[nr][nc] = ng
                h_val          = heuristic(nr, nc, gr, gc)
                parent[nr][nc] = (r, c)
                heapq.heappush(heap, (ng + h_val, ng, nr, nc))

    return None, INF, nodes_explored   # no path

# ── Mark path on grid ───────────────────────────────────────────────────────
def mark_path(grid, path, sr, sc, gr, gc):
    for r, c in path:
        if grid[r][c] not in (START, GOAL):
            grid[r][c] = PATH
    grid[sr][sc] = START
    grid[gr][gc] = GOAL

# ── Print grid ───────────────────────────────────────────────────────────────
def print_grid(grid):
    symbols = {FREE: '.', OBSTACLE: '#', PATH: '*', START: 'S', GOAL: 'G'}
    print("\nGrid Map:  . = Free   # = Obstacle   * = Path   S = Start   G = Goal")
    print("+" + "-" * COLS + "+")
    for row in grid:
        print("|" + "".join(symbols[cell] for cell in row) + "|")
    print("+" + "-" * COLS + "+")

# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    print("*" * 60)
    print("*   UGV Pathfinding - Static Obstacles  (70x70 Grid, A*)   *")
    print("*" * 60)
    print()

    # Density selection
    print("Select obstacle density level:")
    print("  1. Low    (10%)")
    print("  2. Medium (25%)")
    print("  3. High   (40%)")
    while True:
        try:
            choice = int(input("Enter choice [1-3]: "))
            if choice in DENSITY_LEVELS:
                break
            print("  Please enter 1, 2, or 3.")
        except ValueError:
            print("  Invalid input.")

    label, density = DENSITY_LEVELS[choice]

    # Start and goal
    while True:
        try:
            sr, sc = map(int, input("\nEnter Start position  row col  (e.g. 0 0)  [0..69]: ").split())
            gr, gc = map(int, input("Enter Goal  position  row col  (e.g. 69 69) [0..69]: ").split())
            if not (0 <= sr < ROWS and 0 <= sc < COLS and
                    0 <= gr < ROWS and 0 <= gc < COLS):
                print("  Error: Coordinates out of range.")
                continue
            if sr == gr and sc == gc:
                print("  Error: Start and Goal must be different.")
                continue
            break
        except ValueError:
            print("  Invalid input. Enter two integers separated by space.")

    # Build grid
    grid = [[FREE] * COLS for _ in range(ROWS)]
    generate_obstacles(grid, density, sr, sc, gr, gc)
    grid[sr][sc] = START
    grid[gr][gc] = GOAL

    print("\nRunning A* search ...")
    t0                           = time.perf_counter()
    path, path_cost, nodes_expl  = astar(grid, sr, sc, gr, gc)
    elapsed_ms                   = (time.perf_counter() - t0) * 1000

    if path:
        mark_path(grid, path, sr, sc, gr, gc)
        print_grid(grid)
        path_steps = len(path) - 1
        print()
        print("-" * 52)
        print("         Measures of Effectiveness (MOE)")
        print("-" * 52)
        print(f"  Result              : PATH FOUND")
        print(f"  Start Cell          : ({sr} , {sc})")
        print(f"  Goal Cell           : ({gr} , {gc})")
        print(f"  Path Steps (hops)   : {path_steps}")
        print(f"  Path Cost (x10 km)  : {path_cost}")
        print(f"  Nodes Explored      : {nodes_expl}")
        print(f"  Obstacle Density    : {int(density * 100)}%  ({label})")
        print(f"  Computation Time    : {elapsed_ms:.3f} ms")
        print(f"  Grid Size           : {ROWS} x {COLS}")
        print("-" * 52)
    else:
        print(f"\n[RESULT] No path found from ({sr},{sc}) to ({gr},{gc}).")
        print("         Try a lower density or different positions.")

if __name__ == "__main__":
    main()
