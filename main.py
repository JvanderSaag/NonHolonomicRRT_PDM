#Chose the appropriate scenario and pass it as the import module to line 14 and to the run_sim function in line 16.
#Switch animate=False if you do not want to render the simulation.
'''
Scenarios:

ParallelParking_Scenario_ReedShepp
ParallelParking_Scenario_Dubin
Parking_Scenario_ReedsShepp
Parking_Scenario_Dubin
Street_Scenario_ReedsShepp
Street_Scenario_Dubin
'''

from bin.run_controller import run_sim
from Street_Scenario_ReedsShepp import TestScenario

run_sim(TestScenario, "Street_Scenario_ReedsShepp", animate=True)

