from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa
import traci.constants as tc


def generate_routefile():
    random.seed(42)  # make tests reproducible
    N = 3600  # number of time steps
    # demand per second from different directions
    routesN = 13
    ps = [1. / 20] * routesN
    with open("data/route.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="default_type" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" \
guiShape="passenger"/>

        <route id="route0" edges="0to1 1to2 2to3 3to4" />
        <route id="route1" edges="1-to1 1to1+" />
        <route id="route2" edges="1+to1 1to1-" />
        <route id="route3" edges="0to1 1to2 2to3 3to3-" />
        <route id="route4" edges="0to1 1to2 2to2+" />
        <route id="route5" edges="0to1 1to1+" />
        <route id="route6" edges="2+to2 2to1 1to0" />
        <route id="route7" edges="2+to2 2to3 3to4" />
        <route id="route8" edges="3-to3 3to2 2to1 1to0" />
        <route id="route9" edges="3-to3 3to4" />
        <route id="route10" edges="4to3 3to2 2to2+" />
        <route id="route11" edges="4to3 3to2 2to1 1to1+" />
        <route id="route12" edges="4to3 3to2 2to1 1to0" />
        """, file=routes)
        vehNr = 0
        for i in range(N):
            for r_i in range(routesN):
                if random.uniform(0, 1) < ps[r_i]:
                    print('    <vehicle id="route%i_%i" type="default_type" route="route%i" depart="%i" />' % (
                        r_i, vehNr, r_i, i), file=routes)
                    vehNr += 1
        print("</routes>", file=routes)


# The program looks like this
#    <tlLogic id="0" type="static" programID="0" offset="0">
# the locations of the tls are      NESW
#        <phase duration="31" state="GrGr"/> NS has green
#        <phase duration="6"  state="yryr"/> yellow
#        <phase duration="31" state="rGrG"/> EW has green
#        <phase duration="6"  state="ryry"/> yellow
#    </tlLogic>


def run():
    """execute the TraCI control loop"""
    traci.junction.subscribeContext("0", tc.CMD_GET_VEHICLE_VARIABLE, 1000000,
                                    [tc.VAR_SPEED, tc.VAR_ALLOWED_SPEED, tc.VAR_WAITING_TIME])
    step_length = traci.simulation.getDeltaT()

    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        calc_stats(step_length)
        step += 1
    traci.close()
    sys.stdout.flush()


def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options


def calc_stats(step_length):
    sc_results = traci.junction.getContextSubscriptionResults("0")
    halting = 0
    time_loss = 0
    avg_wait_time = 0
    if sc_results:
        rel_speeds = [d[tc.VAR_SPEED] / d[tc.VAR_ALLOWED_SPEED] for d in sc_results.values()]
        # compute values corresponding to summary-output
        running = len(rel_speeds)
        halting = len([1 for d in sc_results.values() if d[tc.VAR_SPEED] < 0.1])
        avg_wait_time = sum([d[tc.VAR_WAITING_TIME] for d in sc_results.values()]) / running
        mean_speed_relative = sum(rel_speeds) / running
        time_loss = (1 - mean_speed_relative) * running * step_length
    print(traci.simulation.getTime(), time_loss, avg_wait_time, halting)


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    generate_routefile()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "data/config.sumocfg",
                 "--tripinfo-output", "tripinfo.xml"])
    run()
    print('FINISHED')
