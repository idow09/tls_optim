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
import numpy as np

routesN = 13
ps = [1. / 20] * routesN
tl1_flows = [ps[2], ps[6] + ps[12] + ps[11] + ps[8], ps[1], ps[0] + ps[3] + ps[4] + ps[5]]
tl1_flows = 90 * np.array(tl1_flows) / sum(tl1_flows)
tl2_flows = [ps[10] + ps[11] + ps[12] + ps[8], ps[0] + ps[3] + ps[4], ps[6] + ps[7]]
tl2_flows = 90 * np.array(tl2_flows) / sum(tl2_flows)
tl3_flows = [ps[10] + ps[11] + ps[12], ps[8] + ps[9], ps[3] + ps[0] + ps[7]]
tl3_flows = 90 * np.array(tl3_flows) / sum(tl3_flows)

tl2phase2weight = {
    "1": [],
    "2": [],
    "3": []
}


def generate_routefile():
    random.seed(42)  # make tests reproducible
    N = 3600  # number of time steps
    # demand per second from different directions
    # routesN = 13
    # ps = [1. / 20] * routesN
    with open("data/route.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="default_type" accel="0.8" decel="4.5" sigma="0.5" 
        length="5" minGap="2.5" maxSpeed="16.67" \
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
                    print(
                        '    <vehicle id="route%i_%i" type="default_type" '
                        'route="route%i" depart="%i" />' % (
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


def verbose(time_in_sec, intgr=True):
    if intgr:
        return "%d sec (= %.2f min)" % (time_in_sec, float(time_in_sec) / 60)
    else:
        return "%.2f sec (= %.2f min)" % (time_in_sec, float(time_in_sec) / 60)


def print_stats(veh_stats, total_time_loss):
    avg_trip_time = sum(d['trip_time'] for d in veh_stats.values()) / float(len(veh_stats))
    min_trip_time = min(d['trip_time'] for d in veh_stats.values())
    max_trip_time = max(d['trip_time'] for d in veh_stats.values())
    avg_wait_time = sum(d['wait_time'] for d in veh_stats.values()) / float(len(veh_stats))
    min_wait_time = min(d['wait_time'] for d in veh_stats.values())
    max_wait_time = max(d['wait_time'] for d in veh_stats.values())
    print("avg_trip_time = ", verbose(avg_trip_time, False))
    print("min_trip_time = ", verbose(min_trip_time))
    print("max_trip_time = ", verbose(max_trip_time))
    print("avg_wait_time = ", verbose(avg_wait_time, False))
    print("min_wait_time = ", verbose(min_wait_time))
    print("max_wait_time = ", verbose(max_wait_time))
    print("total_time_loss = ", verbose(total_time_loss))


def calc_phase(step_in_cycle, phase2weight):
    ylw_time = 6
    net_cycle_time = 90 - ylw_time * len(phase2weight)
    phase2weight = np.array(phase2weight) / sum(phase2weight)
    phase2weight_cum = np.cumsum(phase2weight)
    for phase, weight in enumerate(phase2weight):
        upper = (phase2weight_cum[phase]) * net_cycle_time + ylw_time * phase
        lower = (upper - weight) * net_cycle_time + ylw_time * phase
        if lower < step_in_cycle < upper:
            return phase
    raise RuntimeError('WTF!!!!!!!!!!!!!!!')


def run():
    """execute the TraCI control loop"""
    traci.junction.subscribeContext("0", tc.CMD_GET_VEHICLE_VARIABLE, 1000000,
                                    [tc.VAR_SPEED, tc.VAR_ALLOWED_SPEED,
                                     tc.VAR_WAITING_TIME])
    step_length = traci.simulation.getDeltaT()

    step = 0
    veh_stats = {}
    total_time_loss = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        total_time_loss += calc_step_stats(step_length, veh_stats)

        tls = ["1", "2", "3"]
        for tl in tls:
            traci.trafficlight.setPhase(tl, calc_phase(step % 90, tl2phase2weight[tl]))
        step += 1
    print_stats(veh_stats, total_time_loss)
    traci.close()
    sys.stdout.flush()


def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true",
                          default=False,
                          help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options


def calc_step_stats(step_length, veh_stats):
    sc_results = traci.junction.getContextSubscriptionResults("0")
    halting = 0
    time_loss = 0
    avg_wait_time = 0
    if sc_results:
        rel_speeds = [d[tc.VAR_SPEED] / d[tc.VAR_ALLOWED_SPEED] for d in
                      sc_results.values()]
        # compute values corresponding to summary-output
        running = len(rel_speeds)
        halting = len([1 for d in sc_results.values() if d[tc.VAR_SPEED] < 0.1])
        avg_wait_time = sum(
            [d[tc.VAR_WAITING_TIME] for d in sc_results.values()]) / running
        mean_speed_relative = sum(rel_speeds) / running
        time_loss = (1 - mean_speed_relative) * running * step_length

        for k, d in sc_results.items():
            veh_stats.setdefault(k, {'trip_time': 0, 'wait_time': 0})
            veh_stats[k]['trip_time'] += 1
            if d[tc.VAR_SPEED] < 0.1:
                veh_stats[k]['wait_time'] += 1
    print(traci.simulation.getTime(), time_loss, avg_wait_time, halting)
    return time_loss


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
