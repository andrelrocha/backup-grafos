from grafo import Grafo

class Digrafo(Grafo):
    def __init__(self, num_vertices, ponderado=True):
        """ Inicializa um dígrafo com matriz de adjacência """
        super().__init__(num_vertices, ponderado)

    def adicionar_aresta(self, u, v, peso=1):
        """ Adiciona uma aresta direcionada (u → v) """
        if self.ponderado:
            self.matriz[u][v] = peso  # Apenas u → v
        else:
            self.matriz[u][v] = 1  # Aresta unitária (sem peso)
        self.graus[u] += 1  # Apenas o vértice de origem tem o grau incrementado

    def viz(self, v):
        """ Retorna a vizinhança do vértice v (direcionada) """
        return [i for i in range(1, self.num_vertices + 1) if self.matriz[v][i] < np.inf]

    def bfs(self, v):
        """ Executa busca em largura (BFS) a partir do vértice v """
        d = [np.inf] * (self.num_vertices + 1)
        pi = [None] * (self.num_vertices + 1)
        d[v] = 0
        pi[v] = -1
        fila = [v]
        while fila:
            u = fila.pop(0)
            for w in self.viz(u):
                if d[w] == np.inf:
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
        d = [np.inf] * (self.num_vertices + 1)
        pi = [None] * (self.num_vertices + 1)
        d[v] = 0
        for _ in range(self.num_vertices - 1):
            for u in range(1, self.num_vertices + 1):
                for w in self.viz(u):
                    if d[w] > d[u] + self.w(u, w):
                        d[w] = d[u] + self.w(u, w)
                        pi[w] = u
        return d, pi

    def djikstra(self, v):
        """ Executa o algoritmo de Dijkstra a partir do vértice v """
        d = [np.inf] * (self.num_vertices + 1)
        pi = [None] * (self.num_vertices + 1)
        d[v] = 0
        visitado = [False] * (self.num_vertices + 1)
        for _ in range(self.num_vertices):
            u = min((i for i in range(1, self.num_vertices + 1) if not visitado[i]), key=lambda x: d[x], default=None)
            if u is None: break
            visitado[u] = True
            for w in self.viz(u):
                if not visitado[w] and d[w] > d[u] + self.w(u, w):
                    d[w] = d[u] + self.w(u, w)
                    pi[w] = u
        return d, pi
