# =============================================================================
# Program: UGV Pathfinding - Dynamic Obstacles (Replanning A*)
# =============================================================================

import heapq
import random
import time

ROWS, COLS = 70, 70

FREE, OBSTACLE, PATH, START, GOAL = 0, 1, 2, 3, 4

STRAIGHT = 10
DIAGONAL = 14

DIRECTIONS = [
    (-1,-1,DIAGONAL), (-1,0,STRAIGHT), (-1,1,DIAGONAL),
    (0,-1,STRAIGHT),                  (0,1,STRAIGHT),
    (1,-1,DIAGONAL),  (1,0,STRAIGHT), (1,1,DIAGONAL)
]

# ── Heuristic (same as A*) ─────────────────────────────────────────────────
def heuristic(r1, c1, r2, c2):
    dr, dc = abs(r1-r2), abs(c1-c2)
    return DIAGONAL*min(dr,dc) + STRAIGHT*(max(dr,dc)-min(dr,dc))

# ── A* Function (reusable) ─────────────────────────────────────────────────
def astar(grid, sr, sc, gr, gc):
    INF = float('inf')
    g = [[INF]*COLS for _ in range(ROWS)]
    parent = [[None]*COLS for _ in range(ROWS)]
    closed = [[False]*COLS for _ in range(ROWS)]

    g[sr][sc] = 0
    pq = [(heuristic(sr,sc,gr,gc), 0, sr, sc)]

    while pq:
        f, cost, r, c = heapq.heappop(pq)

        if closed[r][c]:
            continue
        closed[r][c] = True

        if (r,c) == (gr,gc):
            path = []
            cur = (gr,gc)
            while cur:
                path.append(cur)
                cur = parent[cur[0]][cur[1]]
            return path[::-1]

        for dr,dc,w in DIRECTIONS:
            nr, nc = r+dr, c+dc

            if not (0<=nr<ROWS and 0<=nc<COLS):
                continue
            if grid[nr][nc] == OBSTACLE:
                continue

            # prevent diagonal cutting
            if dr!=0 and dc!=0:
                if grid[r+dr][c]==OBSTACLE and grid[r][c+dc]==OBSTACLE:
                    continue

            new_cost = cost + w
            if new_cost < g[nr][nc]:
                g[nr][nc] = new_cost
                parent[nr][nc] = (r,c)
                heapq.heappush(pq, (new_cost + heuristic(nr,nc,gr,gc), new_cost, nr, nc))

    return None

# ── Dynamic obstacle update ────────────────────────────────────────────────
def add_dynamic_obstacles(grid, probability=0.05):
    for _ in range(10):  # randomly add few obstacles
        if random.random() < probability:
            r = random.randint(0, ROWS-1)
            c = random.randint(0, COLS-1)
            if grid[r][c] == FREE:
                grid[r][c] = OBSTACLE

# ── Main dynamic navigation ────────────────────────────────────────────────
def dynamic_navigation(grid, sr, sc, gr, gc):

    current = (sr, sc)
    total_path = [current]
    nodes_replanned = 0

    while current != (gr, gc):

        # Replan path from current position
        path = astar(grid, current[0], current[1], gr, gc)
        nodes_replanned += 1

        if not path:
            print("No path available!")
            return None, nodes_replanned

        # Move step-by-step
        for step in path[1:]:
            # simulate dynamic obstacles appearing
            add_dynamic_obstacles(grid)

            if grid[step[0]][step[1]] == OBSTACLE:
                print("Obstacle detected! Replanning...")
                break

            current = step
            total_path.append(current)

            if current == (gr, gc):
                return total_path, nodes_replanned

    return total_path, nodes_replanned

# ── Print grid ─────────────────────────────────────────────────────────────
def print_grid(grid):
    sym = {FREE:'.', OBSTACLE:'#', PATH:'*', START:'S', GOAL:'G'}
    print("+" + "-"*COLS + "+")
    for row in grid:
        print("|" + "".join(sym[x] for x in row) + "|")
    print("+" + "-"*COLS + "+")

# ── Main ───────────────────────────────────────────────────────────────────
def main():

    grid = [[FREE]*COLS for _ in range(ROWS)]

    sr, sc = 0, 0
    gr, gc = 69, 69

    grid[sr][sc] = START
    grid[gr][gc] = GOAL

    print("Running Dynamic Pathfinding...\n")

    t0 = time.perf_counter()
    path, replans = dynamic_navigation(grid, sr, sc, gr, gc)
    t = (time.perf_counter() - t0)*1000

    if path:
        for r,c in path:
            if grid[r][c] not in (START, GOAL):
                grid[r][c] = PATH

        print_grid(grid)

        print("\n--- MOE ---")
        print("Path Found")
        print("Steps:", len(path)-1)
        print("Replans:", replans)
        print("Time(ms):", round(t,3))
    else:
        print("Failed to reach goal")

if __name__ == "__main__":
    main()