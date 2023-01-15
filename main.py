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
from Street_Scenario_Dubin import TestScenario

run_sim(TestScenario, "Street_Scenario_Dubin", animate=True)

