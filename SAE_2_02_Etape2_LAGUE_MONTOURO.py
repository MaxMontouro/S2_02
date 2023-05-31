#importation des modules et fonctions utilent pour le code
import pandas as pd
from math import pi, cos, acos, sin
import numpy as np
from graphics import GraphWin, Circle, Point, Line

"""différents chemins d'aacès selon les machines utilisées"""
itineraire = pd.read_table('F://Documents/IUT/Annee1/s2/mahs/fichier-itineraires-randonnee.csv',
                         sep = ";",
                             encoding="latin_1")

#itineraire = pd.read_table('C:/Users/maxmo/OneDrive/Documents/IUT/Annee1/S2_02/fichier-itineraires-randonnee.csv', sep = ";", encoding="latin_1")
listepoint = pd.read_table('F://Documents/IUT/Annee1/s2/mahs/listepoint.csv', sep = ";", encoding="latin_1")
matricepoids = pd.read_table('F://Documents/IUT/Annee1/s2/mahs/matpoids.csv', sep = ";", encoding="latin_1")

##Partie C 

#nettoyage du fichier en utilisant le parcourant et ajoutant les uniques sommet dans une liste qui sera transforme en dataframe
Liste = []
for valeur in itineraire.index:
    if itineraire.loc[valeur,'depart '] not in Liste:
        Liste.append(itineraire.loc[valeur,'depart '])
        
    if itineraire.loc[valeur,'arrivée'] not in Liste:
        Liste.append(itineraire.loc[valeur,'arrivée'])
        
#transformation en dataframe
point_unique = pd.DataFrame(Liste)
#renommer la colonne 0 en Coordonnée
point_unique.rename(columns={0 :'Coordonnée'},inplace=True)

#identification de chaque point en un nom unique
point_unique['nom_point'] = None
for ind in point_unique.index:
    point_unique.loc[ind, 'nom_point'] = 'point' + str(ind)

#Réorganisation des colonnes du dataFrame tout_points
lVar = list(point_unique.columns)
nVal = [lVar[1],lVar[0]]
point_unique = point_unique[nVal]

#séparation de la colonne coordonnée en de colonne logitude et latitude
point_unique['longitude'] = point_unique['Coordonnée'].str.split(',').str[0]
point_unique['latitude'] = point_unique['Coordonnée'].str.split(',').str[1]

#jointure des colonnes de itinéraire et point_unique en rapport avec les points de départ
itineraire = pd.merge(itineraire, point_unique,
                      left_on ='depart ', 
                      right_on = 'Coordonnée', 
                      how='inner')

#renommer le nom des colonnes ajouter lors de la jointure
itineraire.columns = ['id_local', 'uuid', 'gestion', 'pratique (été)', 'type_itinéraire',
       'communes_nom', 'depart ', 'arrivée', 'tracé', 'balisage', 'longueur',
       'largeur', 'etat_itineraire', 'foncier', 'remarques', 'type_sol',
       'pdipr_inscription', 'nom_dep', 'coordonnée_depart', 'longitude_depart',
       'latitude_depart']

#jointure des colonnes de itinéraire et point_unique en rapport avec les points d'arrivée
itineraire = pd.merge(itineraire, point_unique,
                      left_on = 'arrivée',
                      right_on = 'Coordonnée',
                      how = 'inner')

#renommer le nom des colonnes ajouter lors de la jointure
itineraire.columns = ['id_local', 'uuid', 'gestion', 'pratique (été)', 'type_itinéraire',
       'communes_nom', 'depart ', 'arrivée', 'tracé', 'balisage', 'longueur',
       'largeur', 'etat_itineraire', 'foncier', 'remarques', 'type_sol',
       'pdipr_inscription', 'nom_dep', 'coordonnée_depart', 'longitude_depart',
       'latitude_depart', 'nom_arr', 'coordonnée_arrivée', 'longitude_arrivée', 'latitude_arrivée']


##PARTIE D.1
dictionnaire_voisin = {}

#initialisation du dictionnaire avec comme couple clé/valeur le nom des points de point_unique/une liste vide
for ind in point_unique.index:
    dictionnaire_voisin[point_unique.loc[ind,"nom_point"]] = []

#Parcours de point_unique et ajout dans le dictionnaire si il s'agit des voisins
for indexX in point_unique.index:
    for indexParcours in point_unique.index:
        if point_unique.loc[indexX, "nom_point"] == itineraire.loc[indexParcours, "nom_dep"]:
            dictionnaire_voisin[point_unique.loc[indexX, "nom_point"]].append(itineraire.loc[indexParcours, "nom_arr"])
        elif point_unique.loc[indexX, "nom_point"] == itineraire.loc[indexParcours, "nom_arr"]:
             dictionnaire_voisin[point_unique.loc[indexX, "nom_point"]].append(itineraire.loc[indexParcours, "nom_dep"])

#ajout de la fonction distanceGPS disponible sur Elearn
def distanceGPS(latA,latB,longA,longB):
# Conversions des latitudes en radians
        ltA=latA/180*pi
        ltB=latB/180*pi
        loA=longA/180*pi
        loB=longB/180*pi
        # Rayon de la terre en mètres 
        RT = 6378137
# angle en radians entre les 2 points
        S = acos(round(sin(ltA)*sin(ltB) + cos(ltA)*cos(ltB)*cos(abs(loB-loA)),14))
# distance entre les 2 points, comptée sur un arc de grand cercle
        return S*RT   

dictionnaire_voisin_distance = {}

#initialisation du dictionnaire avec comme couple clé/valeur le nom des points de point_unique/une liste vide
for ind in point_unique.index:
    dictionnaire_voisin_distance[point_unique.loc[ind,"nom_point"]] = []


#Calcul de distance et ajout du résultat dans une colonne du dataframe appelé "distance"
for ind in itineraire.index :
    itineraire.loc[ind, "distance"] = distanceGPS(float(itineraire.loc[ind, "latitude_depart"]), float(itineraire.loc[ind, "latitude_arrivée"]),
                                                   float(itineraire.loc[ind, "longitude_depart"]), float(itineraire.loc[ind, "longitude_arrivée"]))

#PARTIE B écart
#Calcul de l'écart
itineraire["écart"] = itineraire["longueur"] - itineraire["distance"]

for ind in itineraire.index:
    if itineraire.loc[ind, "écart"] > 5:
        itineraire.loc[ind, "longueur"] = itineraire.loc[ind, "distance"]

#☺Partie D.2
#Parcours de point_unique et ajout dans le dictionnaire si il s'agit des voisins avec la distance (sous forme de tuple)
for indexX in point_unique.index:
    for indexParcours in point_unique.index:
        if point_unique.loc[indexX, "nom_point"] == itineraire.loc[indexParcours, "nom_dep"]:
            dictionnaire_voisin_distance[point_unique.loc[indexX, "nom_point"]].append((itineraire.loc[indexParcours, "nom_arr"], itineraire.loc[indexParcours, "distance"]))
        elif point_unique.loc[indexX, "nom_point"] == itineraire.loc[indexParcours, "nom_arr"]:
             dictionnaire_voisin_distance[point_unique.loc[indexX, "nom_point"]].append((itineraire.loc[indexParcours, "nom_dep"], itineraire.loc[indexParcours, "distance"]))

# Partie D.3
#Création de 2 fonction pour pouvoir accéder aux sommets du graphe par leur nom
def nom(indice):
    nomPoint = point_unique.loc[indice, "nom_point"]
    return nomPoint

def indice_som(nomPoint):
    for ind in range(len(nom_point)):
        if itineraire.iloc[ind].name == nomPoint:
            return ind

#On va créer une liste contenant tous les noms des points
nom_point = list(dictionnaire_voisin_distance.keys())

#Créer une matrice avec la même taille que la liste des points
taille = len(nom_point)
matricePoids = np.zeros((taille, taille))
        

#On remplit la matrice avec les distances du dictionnaire dictionnaire_voisin_distance
for i in range(len(nom_point)):
    for j in range(len(nom_point)):
        if i != j:
            pointDep = nom(i)
            pointArr = nom(j)
            if pointDep in dictionnaire_voisin_distance:
                for (voisin, poids) in dictionnaire_voisin_distance[pointDep]:
                    if voisin == pointArr:
                        matricePoids[i][j] = poids
                        break
                    else:
                        matricePoids[i][j] = 'inf'
        else:
            matricePoids[i][j] = 'inf'
            


#Les différents algos :


#Algorithme de Bellman

def bellman(arret_dep, arret_ariv):
        # Initialiser toutes les distances à l'infini sauf la source à 0
        dist = [float("Inf")] * arret_dep
        dist[arret_ariv] = 0

        for _ in range(arret_dep - 1):
            for u, v, w in arret_dep.graph:
                if dist[u] != float("Inf") and dist[u] + w < dist[v]:
                    dist[v] = dist[u] + w

        # Vérifier s'il y a un cycle de poids négatif
        for u, v, w in arret_dep.graph:
            if dist[u] != float("Inf") and dist[u] + w < dist[v]:
                print("Le graphe contient un cycle de poids négatif")
                return

        # Afficher les distances calculées
        arret_dep.print_solution(dist)




#Algorithme de djikstra
##Celui ci ne marche pas
def djikstra(arret_dep, arret_arriv):
    """
    Renvoie la distance la plus courte entre deux arrêts grâce à l'algorithme de Belmann.
    """

    # Initialisation de la liste des distances
    sommet = indice_som(arret_dep)

    dist = [float('inf')] * len(arret_arriv)
    liste = [float('inf')] * len(arret_arriv)
    pred = [float('inf')] * len(arret_arriv)
    a_traiter = [i for i in range(len(arret_arriv))]

    # Afin d'éviter de passer par le sommet de départ, on l'enlève de la liste des sommets à traiter.
    a_traiter.remove(indice_som(arret_dep))
    pred[sommet] = sommet
    dist[sommet] = 0

    while len(a_traiter) != 0:
        for i in range(len(matricePoids)):
            if i in a_traiter:
                liste[i] = (matricepoids[sommet][i])

        for i in range(len(liste)):
            
            if liste[i] < float('inf'):
                if dist[i] > (dist[sommet] + liste[i]):
                    pred[i] = sommet
                    dist[i] = dist[sommet] + liste[i]
        for i in range(len(matricepoids)):
            liste[i] = (float('inf'))
        for i in a_traiter:
            liste[i] = dist[i]

        print(sommet)
        
#Retourne le sommet de poids minimum de la liste lst
        def extract_min(dist,S):
            min = dist[S[0]]
            index_min_sommet = S[0]
            for i in range (1, len(S)):
                if min > dist[S[i]]:
                    min = dist[S[i]]
                    index_min_sommet = i
            return index_min_sommet
        sommet = extract_min(liste, sommet)
        a_traiter.remove(sommet)
        

    chemin = []
    sommet = indice_som(arret_arriv)

    # Remontée afin d'avoir tous les sommets du chemin
    while sommet != indice_som(arret_dep):
        chemin.append(nom(sommet))
        sommet = pred[sommet]

    chemin.append(arret_dep)
    chemin.reverse()

    return chemin, round(dist[indice_som(arret_arriv)])

##fonction nécéssaire pour le bon fonctionnement de Dijkstra2

def extract_min(dist,S):
    min = dist[S[0]]
    index_min_sommet = S[0]
    for i in range (1, len(S)):
        if min > dist[S[i]]:
            min = dist[S[i]]
            index_min_sommet = i
    return index_min_sommet

def reconstitution_chemin(pointDepart, pointArr, pred) :
    # Reconstitution du chemin à partir des prédécesseurs
    chemin = [pointArr]
    sPred = pred[indice_som(pointArr)]
    chemin.insert(0, sPred)
    while chemin[0] != pointDepart :
        sPred = pred[indice_som(sPred)]
        chemin.insert(0, sPred)
    return chemin

## Celui ci marche 

def Dijkstra2(arret_dep,arret_arriv):
    # Initialisation des tableaux de distance et de prédécesseurs
    dist = [float("inf")] * len(matricepoids) 
    pred = [None] * len(matricepoids) 
    # Liste des sommets non visités
    s = [sommet for sommet in dictionnaire_voisin.keys()]
    # Sommet courant
    sCourant = arret_dep
    dist[indice_som(sCourant)] = 0
    # Suppression du sommet courant de la liste des sommets non visités
    s.remove(sCourant)
    # Boucle principale
    while s != [] :
        # Parcours des voisins du sommet courant
        for i in range(len(dictionnaire_voisin[sCourant])) :
            # Calcul de la distance à partir du sommet courant
            # Si elle est inférieure à la distance précédemment enregistrée, on la met à jour
            if dictionnaire_voisin[sCourant][i][1] + dist[indice_som(sCourant)] < dist[indice_som(dictionnaire_voisin[sCourant][i][0])] :
                dist[indice_som(dictionnaire_voisin[sCourant][i][0])] = dictionnaire_voisin[sCourant][i][1] + dist[indice_som(sCourant)]
                pred[indice_som(dictionnaire_voisin[sCourant][i][0])] = sCourant
        # Extraction du sommet non visité avec la distance minimale
        sCourant = extract_min(dist, s)
        s.remove(sCourant)
    # Si la destination n'a pas de prédécesseur, cela signifie qu'il n'y a pas de chemin entre la source et la destination
    if pred[indice_som(arret_arriv)] == None :
        return dist[indice_som(arret_arriv)], pred[indice_som(arret_arriv)]
    # Sinon, on reconstitue le chemin à partir des prédécesseurs
    return dist[indice_som(arret_arriv)], reconstitution_chemin(arret_dep, arret_arriv, pred)        


#Algorithme de floyd_warshall

def floyd_warshall(arret_dep, arret_arriv):
    # Graphe représenté sous forme de dictionnaire avec les poids des arcs
    graph = {
        'A': {'A': 0, 'B': 3, 'C': 8, 'D': float('inf')},
        'B': {'A': float('inf'), 'B': 0, 'C': float('inf'), 'D': 1},
        'C': {'A': float('inf'), 'B': 4, 'C': 0, 'D': float('inf')},
        'D': {'A': 2, 'B': float('inf'), 'C': -5, 'D': 0}
    }

    # Vérification de la validité des arrêts de départ et d'arrivée
    if arret_dep not in graph or arret_arriv not in graph:
        return float('inf'), []

    # Matrice de distances initiale
    dist = {arret: float('inf') for arret in graph}
    dist[arret_dep] = 0

    # Matrice des prédécesseurs
    pred = {arret: None for arret in graph}

    # Algorithme de Floyd-Warshall
    for k in graph:
        for i in graph:
            for j in graph:
                if dist[i] + graph[i][j] < dist[j]:
                    dist[j] = dist[i] + graph[i][j]
                    pred[j] = i

    # Reconstitution du chemin optimal
    chemin = []
    noeud = arret_arriv
    while noeud != arret_dep:
        chemin.insert(0, noeud)
        noeud = pred[noeud]
        if noeud is None:
            return float('inf'), []

    chemin.insert(0, arret_dep)

    return dist[arret_arriv], chemin

"""distance, chemin = floyd_warshall(arret_dep, arret_arriv)
print("Distance minimale :", distance)
print("Chemin :", chemin)"""


##Récupération des latitudes et longitude de chaque point

ListeLatitude = []
ListeLongitude = []
for index in point_unique.index:
    ListeLatitude.append(float(point_unique.loc[index, "latitude"]))
    ListeLongitude.append(float(point_unique.loc[index, "longitude"]))

##Trier le tableau avec le tri par insertion 
def trierListe(Liste): 
    for i in range(1, len(Liste)): 
        k = Liste[i] 
        j = i-1
        while j >= 0 and k < Liste[j] : 
                Liste[j + 1] = Liste[j] 
                j -= 1
        Liste[j + 1] = k

##ajout des maximum et minimum dans les variables min et max à l'aide de la Liste trier 
trierListe(ListeLatitude)

minLatitude = ListeLatitude[0]
maxLatitude = ListeLatitude[len(Liste) - 1]


##ajout des maximum et minimum dans les variables min et max à l'aide de la Liste trier 
trierListe(ListeLongitude)

minLongitude = ListeLongitude[0]
maxLongitude = ListeLongitude[len(Liste) - 1]

def changementEchelleLatitude(uneLatitude):
    return (uneLatitude-45)*2500

def changementEchelleLongitude(uneLongitude):
    return (uneLongitude-6.3)*2500

def affichageLignePoints():
    fenetre = GraphWin("Itineraire des points",900, 900)
    
    ##affichage des tout les points
    for index in point_unique.index:
        point = Circle(Point(changementEchelleLongitude(float(point_unique.loc[index, "longitude"])), 
                             changementEchelleLatitude(float(point_unique.loc[index, "latitude"]))), 2)
        fenetre.update()
        point.draw(fenetre)
        
    for index in itineraire.index:
        
        pointDeb = (changementEchelleLongitude(float(itineraire.loc[index, "longitude_depart"])),
                    changementEchelleLatitude(float(itineraire.loc[index, "latitude_depart"])))

        pointArr = (changementEchelleLongitude(float(itineraire.loc[index, "longitude_arrivée"])),
                    changementEchelleLatitude(float(itineraire.loc[index, "latitude_arrivée"])))

        plusCourtChemin = djikstra(pointDeb, pointArr)
        plusCourtChemin.setOutline('red')
        plusCourtChemin.draw(fenetre)
        dessineLigne = Line(Point(pointDeb[0], pointDeb[1]), Point(pointArr[0], pointArr[1]))
        dessineLigne.draw(fenetre)  
        
    fenetre.getMouse()
    fenetre.close()
               
        
print(affichageLignePoints())

"""     plusCourtChemin = djikstra(pointDeb, pointArr)
        plusCourtChemin.setOutline('red')
        plusCourtChemin.draw(fenetre)"""