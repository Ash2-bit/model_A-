import tkinter as tk
from tkinter import ttk, messagebox
import heapq
from collections import deque

# ===================== Algoritma Pencarian =====================
def bfs(graph, start, goal):
    queue = deque([[start]])
    visited = set()

    while queue:
        path = queue.popleft()
        node = path[-1]

        if node == goal:
            return path

        if node not in visited:
            visited.add(node)
            for neighbor, _ in graph.get(node, []):
                if neighbor not in path:
                    new_path = list(path)
                    new_path.append(neighbor)
                    queue.append(new_path)
    return None

def dfs(graph, start, goal, path=None, visited=None):
    if path is None:
        path = [start]
    if visited is None:
        visited = set()

    visited.add(start)
    if start == goal:
        return path

    for neighbor, _ in graph.get(start, []):
        if neighbor not in visited:
            new_path = dfs(graph, neighbor, goal, path + [neighbor], visited)
            if new_path:
                return new_path
    return None

def dijkstra(graph, start, goal):
    queue = [(0, start, [start])]
    visited = set()

    while queue:
        cost, node, path = heapq.heappop(queue)

        if node == goal:
            return path, cost

        if node in visited:
            continue
       
        visited.add(node)

        for neighbor, weight in graph.get(node, []):
            if neighbor not in visited:
                heapq.heappush(queue, (cost + weight, neighbor, path + [neighbor]))

    return None, float('inf')

def ucs(graph, start, goal):
    return dijkstra(graph, start, goal)

def greedy_best_first(graph, start, goal, heuristic):
    queue = [(heuristic(start, goal), start, [start])]
    visited = set()

    while queue:
        _, node, path = heapq.heappop(queue)

        if node == goal:
            return path

        if node in visited:
            continue

        visited.add(node)

        for neighbor, _ in graph.get(node, []):
            if neighbor not in visited:
                heapq.heappush(queue, (heuristic(neighbor, goal), neighbor, path + [neighbor]))

    return None

def a_star(graph, start, goal, heuristic):
    open_set = []
    heapq.heappush(open_set, (0, start))

    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    came_from = {}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1], g_score[goal]

        for neighbor, weight in graph.get(current, []):
            tentative_g_score = g_score[current] + weight

            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))

    return None, float('inf')

def heuristic(node, goal):
    return abs(hash(node) - hash(goal)) % 100

def calculate_cost(distance):
    return (distance // 10) * 1000

# ===================== Fungsi Utama =====================
def find_path(algorithm):
    start_node = start_combobox.get()
    goal_node = goal_combobox.get()

    if start_node == goal_node:
        result_label.config(text="Titik awal dan tujuan tidak boleh sama.")
        return

    if start_node not in graph or goal_node not in graph:
        messagebox.showerror("Error", "Titik awal atau tujuan tidak ditemukan dalam graf.")
        return

    result = ""
    if algorithm in ("BFS", "All"):
        path = bfs(graph, start_node, goal_node)
        result += f"\nBFS: {' -> '.join(path) if path else 'Tidak ditemukan'}"

    if algorithm in ("DFS", "All"):
        path = dfs(graph, start_node, goal_node)
        result += f"\nDFS: {' -> '.join(path) if path else 'Tidak ditemukan'}"

    if algorithm in ("Dijkstra", "All"):
        path, cost = dijkstra(graph, start_node, goal_node)
        price = calculate_cost(cost)
        result += f"\nDijkstra: {' -> '.join(path) if path else 'Tidak ditemukan'} (Jarak: {cost} m, Biaya: {price} Rp)"

    if algorithm in ("UCS", "All"):
        path, cost = ucs(graph, start_node, goal_node)
        price = calculate_cost(cost)
        result += f"\nUCS: {' -> '.join(path) if path else 'Tidak ditemukan'} (Jarak: {cost} m, Biaya: {price} Rp)"

    if algorithm in ("Greedy", "All"):
        path = greedy_best_first(graph, start_node, goal_node, heuristic)
        result += f"\nGreedy: {' -> '.join(path) if path else 'Tidak ditemukan'}"

    if algorithm in ("A*", "All"):
        path, cost = a_star(graph, start_node, goal_node, heuristic)
        price = calculate_cost(cost)
        result += f"\nA*: {' -> '.join(path) if path else 'Tidak ditemukan'} (Jarak: {cost} m, Biaya: {price} Rp)"

    result_label.config(text=result.strip())

def reset_selection():
    start_combobox.set("")
    goal_combobox.set("")
    result_label.config(text="")

# ===================== Data Graf =====================
graph = {
    "Pintu Gerbang Depan": [("Pasca Hukum", 200)],
    "Pasca Hukum": [("Pintu Gerbang Depan", 200), ("MAKSI (Ged C)", 400), ("Gedung F", 500)],
    "MAKSI (Ged C)": [("Pasca Hukum", 400), ("Ged. B", 300)],
    "Ged. B": [("MAKSI (Ged C)", 300), ("Ged. A", 200)],
    "Ged. A": [("Ged. B", 200), ("Masjid UNIB", 100)],
    "Masjid UNIB": [("Ged. A", 100)],
    "Gedung F": [("Pasca Hukum", 500), ("Lab. Hukum", 300), ("Ged. I", 200), ("Ged. J", 200), ("Dekanat Pertanian", 200)],
    "Lab. Hukum": [("Gedung F", 100)],
    "Ged. I": [("Gedung F", 150), ("Ged. MM", 150)],
    "Ged. MM": [("Ged. I", 200), ("Ged. MPP", 200)],
    "Ged. MPP": [("Ged. MM", 100), ("Ged. UPT B. Inggris", 100)],
    "Ged. J": [("Gedung F", 100), ("Ged. UPT B. Inggris", 100)],
    "Ged. UPT B. Inggris": [("Ged. J", 100), ("REKTORAT", 150)],
    "Dekanat Pertanian": [("Gedung F", 150), ("Ged. T", 150)],
    "Ged. T": [("Dekanat Pertanian", 150), ("Ged. V", 150)],
    "Ged. V": [("Ged. T", 150), ("Ged. Renper", 150), ("REKTORAT", 150), ("UPT Puskom", 150)],
    "Ged. Renper": [("Ged. V", 150), ("Lab. Agro", 150)],
    "Lab. Agro": [("Ged. Renper", 150), ("Ged. Basic Sains", 150)],
    "Ged. Basic Sains": [("Lab. Agro", 150), ("GKB I", 150), ("Dekanat MIPA", 150)],
    "UPT Puskom": [("Ged. V", 150), ("GKB I", 150)],
    "REKTORAT": [("Ged. UPT B. Inggris", 150), ("Ged. V", 150), ("Dekanat FISIP", 150)],
    "Dekanat FISIP": [("REKTORAT", 150), ("Pintu Gerbang", 150), ("GKB II", 150)],
    "Pintu Gerbang": [("Dekanat FISIP", 150), ("Dekanat Teknik", 150)],
    "Dekanat Teknik": [("Pintu Gerbang", 150), ("Gedung Serba Guna (GSG)", 150)],
    "Gedung Serba Guna (GSG)": [("Dekanat Teknik", 150), ("Stadion Olahraga", 150), ("GKB III", 150), ("Dekanat FKIP", 150)],
    "GKB I": [("UPT Puskom", 150), ("GKB II", 150), ("Ged. Basic Sains", 150)],
    "GKB II": [("GKB I", 150), ("Dekanat FKIP", 150), ("Dekanat FISIP", 150)],
    "Dekanat FKIP": [("GKB II", 150), ("Gedung Serba Guna (GSG)", 150)],
    "GKB V": [("PKM", 150), ("PSPD", 150)],
    "Stadion Olahraga": [("GKB III", 150), ("PSPD", 150)],
    "GKB III": [("Gedung Serba Guna (GSG)", 150)],
    "PKM": [("GKB V", 150)],
    "PSPD": [("GKB V", 150), ("Stadion Olahraga", 150)],
    "Dekanat MIPA": [("Ged. Basic Sains", 150)]
}

# ===================== GUI =====================
root = tk.Tk()
root.title("Pencarian Jalur GPS Kampus")
root.geometry("700x550")

tk.Label(root, text="Titik Awal").pack()
start_combobox = ttk.Combobox(root, values=list(graph.keys()), width=60)
start_combobox.pack()

tk.Label(root, text="Titik Tujuan").pack()
goal_combobox = ttk.Combobox(root, values=list(graph.keys()), width=60)
goal_combobox.pack()

for algo in ["BFS", "DFS", "Dijkstra", "UCS", "Greedy", "A*", "All"]:
    tk.Button(root, text=f"Cari Jalur {algo}", command=lambda a=algo: find_path(a)).pack(pady=2)

tk.Button(root, text="Reset", command=reset_selection, bg="red", fg="white").pack(pady=5)

result_label = tk.Label(root, text="", justify="left", anchor="w", wraplength=650)
result_label.pack(pady=10, fill=tk.BOTH, expand=True)

root.mainloop()