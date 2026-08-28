from algorithms.problems import SearchProblem
import algorithms.utils as utils
from world.game import Directions
from algorithms.heuristics import nullHeuristic


def tinyDiagnosticSearch(problem: SearchProblem):
    """
    Returns a hard-coded sequence of moves for the tinyDiagnostic layout.
    For any other station layout, the sequence of moves will be incorrect.
    """
    s = Directions.SOUTH
    e = Directions.EAST
    return [s, e, s, e, e, e, e, s, e, e, s, s, e, s, s, e, s, e, e, e, e, e, e, e]


def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    pila = utils.Stack()
    visitados = set()
    
    inicio = problem.getStartState()
    pila.push((inicio, []))
    
    while not pila.isEmpty():
        lugar, accion = pila.pop()
        if problem.isGoalState(lugar):
            return accion
        
        if lugar not in visitados:
            visitados.add(lugar)
            for sucesor, accion_sucesor, costo_no_importa in problem.getSuccessors(lugar):
                if sucesor not in visitados:
                    camino = accion + [accion_sucesor]
                    pila.push((sucesor, camino))
    
    return []


def breadthFirstSearch(problem: SearchProblem):
    """
    Search the shallowest nodes in the search tree first.
    """
    cola = utils.Queue()
    visitados = set()
    
    inicio = problem.getStartState()
    cola.push((inicio, []))
    
    while not cola.isEmpty():
        lugar, accion = cola.pop()
        if problem.isGoalState(lugar):
            return accion
        
        if lugar not in visitados:
            visitados.add(lugar)
            for sucesor, accion_sucesor, costo_no_importa in problem.getSuccessors(lugar):
                if sucesor not in visitados:
                    camino = accion + [accion_sucesor]
                    cola.push((sucesor, camino))
    
    return []


def uniformCostSearch(problem: SearchProblem):
    """
    Search the node of least total cost first.
    debo:
    1. crear una cola de prioridad
    2. creal una estrucutra que me permita conocer los nodos visitados
    3. tomar el nodo del que arranco y meterlo a la cola con costo 0 (es de donde arranque no consumo energia par eso)
    4. inicio a recorrer la cola de prioridad hasta que este vacia (o si encuentro el objetivo me dentego)
    5. me aseguro de que cada nodo que visite sea el objetivo, si lo es retorno el camino.
    6. si no es, añado los sucesores del nodo junto su costo acumulado
    """
    cola_de_prioridad= utils.PriorityQueue()
    visitados= set()
    inicio = problem.getStartState()
    
    cola_de_prioridad.push((inicio, [], 0), 0)
    while cola_de_prioridad.isEmpty()!= True:
        nodo_actual,pasos_hacia_el_objetivo,cotso_actual= cola_de_prioridad.pop()
        if nodo_actual not in visitados:
            visitados.add(nodo_actual)
            if problem.isGoalState(nodo_actual):
                return pasos_hacia_el_objetivo
            
            for sucesor, accion_sucesor, costo_del_paso in problem.getSuccessors(nodo_actual):
                if sucesor not in visitados:
                    camino= pasos_hacia_el_objetivo + [accion_sucesor]
                    costo_acumulado_pasos=cotso_actual + costo_del_paso
                    cola_de_prioridad.push((sucesor, camino, costo_acumulado_pasos),costo_acumulado_pasos)
    return[]


def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    
    1. frontera: PQ ordenada por h(n)
    2. alcanzados: los nodos con su g(n) ya expandidos
    3. expandir la frontera y revisar los hijos (actualizar si un nodo alcanzado tiene g(n) menor)
    
    """
    nodo = problem.getStartState() # nodo = (position, hasKit, pendingSystems)
    frontera = utils.PriorityQueue()
    alcanzados = {} # {nodo: g(nodo)}
    
    frontera.push((nodo, [], 0), 0 + heuristic(nodo, problem)) # item: (nodo, acciones, g(nodo) + h(nodo))
    
    while not frontera.isEmpty():
        nodo, acciones, g = frontera.pop()
        
        if problem.isGoalState(nodo):
            return acciones
        
        hijos = problem.getSuccessors(nodo)
        for hijo in hijos:
            
            nodo_hijo = hijo[0]
            nodo_direccion = hijo[1]
            nodo_costo = hijo[2]
            
            if (nodo_hijo not in alcanzados) or (g + nodo_costo < alcanzados[nodo_hijo]):
                alcanzados[nodo_hijo] = g + nodo_costo
                frontera.push((nodo_hijo, acciones + [nodo_direccion], alcanzados[nodo_hijo]), heuristic(nodo_hijo, problem) + alcanzados[nodo_hijo])
        
    return []
        


# Abbreviations (you can use them for the -f option in main.py)
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
