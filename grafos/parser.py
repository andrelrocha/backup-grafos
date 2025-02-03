from digrafo import Digrafo

class DigrafoParser:
    def __init__(self):
        self.digrafo = None

    def parse(self, arquivo):
        """ Faz a leitura do arquivo DIMACS e cria o Digrafo """
        with open(arquivo, 'r') as file:
            linhas = file.readlines()

        # Busca pelas informações no cabeçalho do arquivo
        num_vertices = 0
        num_arestas = 0

        for linha in linhas:
            if linha.startswith("p sp"):
                partes = linha.split()
                num_vertices = int(partes[1])
                num_arestas = int(partes[2])
                break

        # Cria o digrafo com o número de vértices
        self.digrafo = Digrafo(num_vertices)

        # Lê as arestas do arquivo
        for linha in linhas:
            if linha.startswith('a'):
                partes = linha.split()
                u = int(partes[1])
                v = int(partes[2])
                peso = int(partes[3]) if len(partes) > 3 else 1  # Pega o peso, se existir
                self.digrafo.adicionar_aresta(u, v, peso)

        return self.digrafo
