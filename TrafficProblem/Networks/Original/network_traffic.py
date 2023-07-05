main_entrances = ["5268187","-667045876#2","25555425#0","377120746#0","282754872"]
main_exits = ["-1101393189","667045876#2", "-25555425#0","-377120746#0","-1160592981"]

minor_entrances = ["-23442946","567533018","23477079","-29505870#1","4250965","-674616889","23565410#1","-567538334#1"]
minor_exits = ["23442946","-18921815#1","-23477079","29505870#1","-674628384","23516537","-23565410#1","567538334#1"]
minor_passthrough_parameter = 0.5 # Percentage of cars which travel through the town without stopping

city_centre = [4120, 3000] # x,y coordinates of centre
retail_park = ["25357015","-25357015"] # Entrance, Exit
industrial_estate = ["-25554584","25554584"] # Entrance, Exit


residential_0 = [4000,5000] # x,y of Tyherington
residential_1 = [2555,3500] # x,y of Upton priory
residential_2 = [2650,2050] # x,y of Ivy Meade
residential_3 = [3600,1300] # x,y of Moss Rose
residential_4 = [5250,2600] # x,y of Hurdsfield

# to residential, to city centre, to retail, to industrial, to minor road, to major road
route_distribution_from_residential_probabilities = [0.2, 0.25, 0.25, 0.05, 0.1, 0.15]
route_distribution_from_city_probabilities = [0.35,0,0.3,0.05,0.1,0.2]
route_distribution_from_retail_probabilities = [0.4, 0.2, 0, 0, 0.15, 0.25]
route_distribution_from_industrial_probabilities = [0.2, 0.1, 0.1, 0, 0.2, 0.4]
route_distribution_from_minor_road_probabilities = [0.3, 0.2, 0.2, 0.1, 0.1, 0.1]
route_distribution_from_major_road_probabilities = [0.2, 0.1, 0.1, 0.1, 0.15, 0.35]

road_sensors = ["23031666#9", "-23031666#9", "1160592977", "-1160592977", "23031661#7", "-111866916#0", "-28673568#1",
                "28673568#1", "28673576#3", "-28673576#3", "-25643530", "25643530", "-25100554#1", "25100554#1",
                "28198325#2", "-28198325#2", "25147116#0", "-25147116#0"]

# action_set = ["23050638#1","24183239#3","201184512#6"]
action_set = ["23050638#1","24183239#3","201184512#6","4250971#4","1133541328#2","1133541331#2","28673575","29068343#0"]