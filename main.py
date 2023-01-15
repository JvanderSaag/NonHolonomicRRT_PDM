#ParallelParking_Scenario_ReedShepp
#ParallelParking_Scenario_Dubin
#Parking_Scenario_ReedsShepp
#Parking_Scenario_Dubin
#Street_Scenario_ReedsShepp
#Street_Scenario_Dubin

from bin.run_controller import run_sim

from Street_Scenario_Dubin import TestScenario
from bin.csv_utils import csv_keys

run_sim(TestScenario, csv_keys["Street_Scenario_Dubin"])

