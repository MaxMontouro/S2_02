import pandas as pd
from math import pi, cos, acos, sin
import numpy as np
#import os

"""différents chemins d'aacès selon les machines utilisées"""
"""itineraire = pd.read_table('F://Documents/IUT/Annee1/s2/mahs/fichier-itineraires-randonnee.csv',
                         sep = ";",
                         encoding="latin_1")"""

itineraire = pd.read_table('C:/Users/maxmo/OneDrive/Documents/IUT/Annee1/S2_02/fichier-itineraires-randonnee.csv', sep = ";", encoding="latin_1")

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