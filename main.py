'''
Scenarios:

ParallelParking_Scenario_ReedShepp
ParallelParking_Scenario_Dubin
Parking_Scenario_ReedsShepp
Parking_Scenario_Dubin
Street_Scenario_ReedsShepp
Street_Scenario_Dubinrr
'''

from bin.run_controller import run_sim
from Parking_Scenario_Dubin import TestScenario

run_sim(TestScenario, "Parking_Scenario_Dubin", animate=True)

