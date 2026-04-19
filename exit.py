import traci
import numpy as np

# start SUMO
traci.start(["sumo-gui", "-c", "ateneo gui.sumocfg"])

veh_counter = 0

# -------------------------
# TRACKING
# -------------------------
vehicle_start_time = {}
vehicle_last_edge = {}

test_start_time = {}
test_exit_map = {}

# store ALL travel times per session
session_times = {}

# -------------------------
# FREE-FLOW TIMES (GIVEN)
# -------------------------
free_flow_times = {
    "g1": 510,
    "g2": 385,
    "g3": 365
}

# -------------------------
# EDGE → RETURN ROUTE MAP
# -------------------------
edge_to_route = {
    "-97366424#1": "canteen-exit",
    "97366427": "gym-exit",
    "97366431#6": "jhs-exit",
    "97366429#0": "jhs-exit",
    "97366432#2": "jhs-exit",
    "97366429#0.53": "grand-exit",
    "1349985629#0": "grand-g2",
    "97366429#2": "gs-exit",
    "97366431#3": "gs-exit",
    "E7": "park1-exit",
    "E13": "park2-exit",
    "E9": "park2-exit",
    "97366431#1": "pre-exit",
    "97366429#3": "pre-exit",
    "-97366412": "shs-exit"
}

# =========================
# SIMULATION LOOP
# =========================
while traci.simulation.getMinExpectedNumber() > 0:

    traci.simulationStep()
    current_time = traci.simulation.getTime()

    vehicles = traci.vehicle.getIDList()

    # -------------------------
    # DETECT SPAWN (ALL VEHICLES)
    # -------------------------
    for v in vehicles:
        vehicle_last_edge[v] = traci.vehicle.getRoadID(v)

        if v not in vehicle_start_time:
            vehicle_start_time[v] = current_time

        # test vehicle tracking
        if v.startswith("test_") and v not in test_start_time:
            test_start_time[v] = current_time

    # -------------------------
    # ARRIVED VEHICLES
    # -------------------------
    arrived = traci.simulation.getArrivedIDList()

    for veh in arrived:

        if veh not in vehicle_last_edge:
            continue

        edge = vehicle_last_edge[veh]

        if edge not in edge_to_route:
            continue

        route = edge_to_route[edge]

        # =====================================
        # TEST VEHICLE EXIT (CUSTOM)
        # =====================================
        if veh.startswith("test_"):

            if "g1" in veh:
                route = "gym-g1"
            elif "g2" in veh:
                route = "gym-g2"
            elif "g3" in veh:
                route = "pre-g3"

            newVehID = veh + "_exit"

            traci.vehicle.add(
                vehID=newVehID,
                routeID=route,
                typeID="DEFAULT_VEHTYPE",
                depart=current_time + 1,
            )

            test_exit_map[newVehID] = veh

        # =====================================
        # NORMAL VEHICLES (UNCHANGED EXIT)
        # =====================================
        else:
            newVehID = "return_" + str(veh_counter)

            traci.vehicle.add(
                vehID=newVehID,
                routeID=route,
                typeID="DEFAULT_VEHTYPE",
                depart=current_time + 2,
            )

            veh_counter += 1

        # =====================================
        # STORE TRAVEL TIME FOR ALL FLOW VEHICLES
        # =====================================
        if veh in vehicle_start_time:

            total_time = current_time - vehicle_start_time[veh]

            # detect group + session from ID
            if "g1Cars" in veh:
                group = "g1"
            elif "g2Cars" in veh:
                group = "g2"
            elif "g3Cars" in veh:
                group = "g3"
            else:
                continue

            # extract session number
            # expects something like g1Cars_s3_###
            if "_s" in veh:
                session_part = veh.split("_s")[1]
                session_number = session_part.split(".")[0]  # removes .142 etc
                key = f"{group}_s{session_number}"
            else:
                continue

            if key not in session_times:
                session_times[key] = []

            session_times[key].append(total_time)

    # -------------------------
    # TEST VEHICLE FINAL TIME
    # -------------------------
    for veh in arrived:
        if veh in test_exit_map:
            original = test_exit_map[veh]

            if original in test_start_time:
                total_time = current_time - test_start_time[original]
                print(f"{original} TOTAL TRAVEL TIME: {total_time:.2f} seconds")

traci.close()

# =========================
# SESSION METRICS (ALL VEHICLES)
# =========================
print("\n==============================")
print("SESSION-BASED METRICS (ALL VEHICLES)")
print("==============================")

for key in sorted(session_times.keys(), key=lambda x: (x.split("_")[0], int(x.split("_s")[1]))):

    times = session_times[key]

    if len(times) == 0:
        continue

    mean_tt = np.mean(times)
    tt95 = np.percentile(times, 95)

    group = key.split("_")[0]
    free_flow = free_flow_times[group]

    pti = tt95 / free_flow
    bti = ((tt95 - mean_tt) / mean_tt) * 100

    print(f"\nSession: {key}")
    print(f"  Vehicles  = {len(times)}")
    print(f"  Mean TT   = {mean_tt:.2f} s")
    print(f"  TT95      = {tt95:.2f} s")
    print(f"  PTI       = {pti:.2f}")
    print(f"  BTI       = {bti:.2f}%")
