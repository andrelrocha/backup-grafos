import math

class Grafo:
    def __init__(self, num_vertices, ponderado=True):
        """ Inicializa um grafo com lista de adjacência """
        self.num_vertices = num_vertices
        self.ponderado = ponderado
        self.lista_adjacencia = {i: [] for i in range(1, num_vertices + 1)}  # Lista de adjacência
        self.graus = [0] * (num_vertices + 1)  # Grau de cada vértice

    def adicionar_aresta(self, u, v, peso=1):
        """ Adiciona uma aresta não direcionada (u ↔ v) """
        if self.ponderado:
            self.lista_adjacencia[u].append((v, peso))
            self.lista_adjacencia[v].append((u, peso))  # Simetria para grafo não direcionado
        else:
            self.lista_adjacencia[u].append((v, 1))
            self.lista_adjacencia[v].append((u, 1))  # Simetria
        self.graus[u] += 1
        self.graus[v] += 1

    def n(self):
        """ Retorna o número de vértices do grafo """
        return self.num_vertices

    def m(self):
        """ Retorna o número de arestas do grafo """
        return sum(len(vizinhos) for vizinhos in self.lista_adjacencia.values()) // 2

    def viz(self, v):
        """ Retorna a vizinhança do vértice v """
        return [w for w, _ in self.lista_adjacencia[v]]

    def d(self, v):
        """ Retorna o grau do vértice v """
        return self.graus[v]

    def w(self, u, v):
        """ Retorna o peso da aresta uv """
        for w, peso in self.lista_adjacencia[u]:
            if w == v:
                return peso
        return math.inf  # Retorna infinito caso não haja aresta entre u e v

    def mind(self):
        """ Retorna o menor grau presente no grafo """
        return min(g for g in self.graus[1:] if g > 0)

    def maxd(self):
        """ Retorna o maior grau presente no grafo """
        return max(self.graus[1:])

    def bfs(self, v):
        """ Executa busca em largura (BFS) a partir do vértice v """
        d = [math.inf] * (self.num_vertices + 1)
        pi = [None] * (self.num_vertices + 1)
        d[v] = 0
        pi[v] = -1
        fila = [v]
        while fila:
            u = fila.pop(0)
            for w in self.viz(u):
                if d[w] == math.inf:
                    d[w] = d[u] + 1
                    pi[w] = u
                    fila.append(w)
        return d, pi

    def dfs(self, v):
        """ Executa busca em profundidade (DFS) a partir do vértice v """
        pi = [None] * (self.num_vertices + 1)
        ini = [None] * (self.num_vertices + 1)
        fim = [None] * (self.num_vertices + 1)
        tempo = [0]  # Usado para controlar o tempo de início e fim
        
        def dfs_visit(u):
            tempo[0] += 1
            ini[u] = tempo[0]
            for w in self.viz(u):
                if pi[w] is None:
                    pi[w] = u
                    dfs_visit(w)
            tempo[0] += 1
            fim[u] = tempo[0]

        pi[v] = -1
        dfs_visit(v)
        return pi, ini, fim

    def bf(self, v):
        """ Executa o algoritmo de Bellman-Ford a partir do vértice v """
        d = [math.inf] * (self.num_vertices + 1)
        pi = [None] * (self.num_vertices + 1)
        d[v] = 0
        for _ in range(self.num_vertices - 1):
            for u in range(1, self.num_vertices + 1):
                for w, peso in self.lista_adjacencia[u]:
                    if d[w] > d[u] + peso:
                        d[w] = d[u] + peso
                        pi[w] = u
        return d, pi

    def djikstra(self, v):
        """ Executa o algoritmo de Dijkstra a partir do vértice v """
        d = [math.inf] * (self.num_vertices + 1)
        pi = [None] * (self.num_vertices + 1)
        d[v] = 0
        visitado = [False] * (self.num_vertices + 1)
        for _ in range(self.num_vertices):
            u = min((i for i in range(1, self.num_vertices + 1) if not visitado[i]), key=lambda x: d[x], default=None)
            if u is None: break
            visitado[u] = True
            for w, peso in self.lista_adjacencia[u]:
                if not visitado[w] and d[w] > d[u] + peso:
                    d[w] = d[u] + peso
                    pi[w] = u
        return d, pi
