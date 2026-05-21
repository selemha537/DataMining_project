from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# =========================
# Données du problème
# =========================

def create_data_model():

    data = {}

    # Matrice des distances
    # 0 = Port
    # 1 = Marché 1
    # 2 = Marché 2
    # 3 = Marché 3

    data['distance_matrix'] = [

        [0, 20, 30, 10],

        [20, 0, 15, 25],

        [30, 15, 0, 18],

        [10, 25, 18, 0],
    ]

    # Demandes des marchés (kg)
    data['demands'] = [0, 200, 300, 250]

    # Nombre de camions
    data['num_vehicles'] = 2

    # Capacités des camions
    data['vehicle_capacities'] = [500, 500]

    # Dépôt = Port
    data['depot'] = 0

    return data


# =========================
# Création des données
# =========================

data = create_data_model()

# =========================
# Gestion des index
# =========================

manager = pywrapcp.RoutingIndexManager(

    len(data['distance_matrix']),

    data['num_vehicles'],

    data['depot']
)

# =========================
# Création du modèle
# =========================

routing = pywrapcp.RoutingModel(manager)

# =========================
# Fonction distance
# =========================

def distance_callback(from_index, to_index):

    from_node = manager.IndexToNode(from_index)

    to_node = manager.IndexToNode(to_index)

    return data['distance_matrix'][from_node][to_node]


# Enregistrement de la distance

transit_callback_index = routing.RegisterTransitCallback(
    distance_callback
)

# Fonction objectif :
# minimiser la distance

routing.SetArcCostEvaluatorOfAllVehicles(
    transit_callback_index
)

# =========================
# Fonction demande
# =========================

def demand_callback(from_index):

    from_node = manager.IndexToNode(from_index)

    return data['demands'][from_node]

# Enregistrement de la demande

demand_callback_index = routing.RegisterUnaryTransitCallback(
    demand_callback
)

# =========================
# Contraintes de capacité
# =========================

routing.AddDimensionWithVehicleCapacity(

    demand_callback_index,

    0,

    data['vehicle_capacities'],

    True,

    'Capacity'
)

# =========================
# Paramètres de recherche
# =========================

search_parameters = pywrapcp.DefaultRoutingSearchParameters()

search_parameters.first_solution_strategy = (

    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
)

# =========================
# Résolution
# =========================

solution = routing.SolveWithParameters(search_parameters)

# =========================
# Affichage des résultats
# =========================

if solution:

    total_distance = 0

    for vehicle_id in range(data['num_vehicles']):

        index = routing.Start(vehicle_id)

        print(f"\nCamion {vehicle_id + 1} :")

        route_distance = 0

        while not routing.IsEnd(index):

            node = manager.IndexToNode(index)

            print(node, end=" -> ")

            previous_index = index

            index = solution.Value(
                routing.NextVar(index)
            )

            route_distance += routing.GetArcCostForVehicle(

                previous_index,

                index,

                vehicle_id
            )

        print(manager.IndexToNode(index))

        print("Distance du camion :", route_distance, "km")

        total_distance += route_distance

    print("\nDistance totale =", total_distance, "km")