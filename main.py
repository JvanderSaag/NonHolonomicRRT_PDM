#ParallelParking_Scenario_ReedShepp # ReedsShepp_19.29_1k
#ParallelParking_Scenario_Dubin # Dubin_04.26_5k
#Parking_Scenario_ReedsShepp # ReedsShepp_22.59_2k
#Parking_Scenario_Dubin # Dubins_5.25_20k
#Street_Scenario_ReedsShepp # ReedsShepp_12.56_2k
#Street_Scenario_Dubin # Dubins_39.54_20k

#Scenario1
#simple_Scenario
from bin.run_controller import run_sim
from Street_Scenario_Dubin import TestScenario

run_sim(TestScenario, 'Dubins_39.54_20k')