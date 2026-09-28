# UE5 Drone Simulator — notes for Claude

Quadrotor flight simulator. Dynamics and control live in engine-independent headers (`Source/Drone_Simulater/FlightCore/`);
UE5 (`DroneActor.*`) only drives, draws and logs them. README.md (Korean) is the source of truth for results and the dev log.

## Verify (headless, no UE needed)
```
cd Tools/FlightSim
c++ -std=c++17 -O2 -Wall -Wextra -Wshadow -I ../../Source/Drone_Simulater sim_main.cpp -o flightsim   # or: python -m ziglang c++ ...
./flightsim out && python plot.py out ../../docs/img
```
40 checks, exit code 1 on any failure, fixed seeds (output is deterministic). Takes ~30 s.

## Open task: first UE build of v3 (commit 27733d8)
The v3 UE changes were written on a PC without UE and have never been compiled. On the UE 5.7 PC:
1. Build the editor target. Fix any errors in `DroneActor.cpp/.h` or `FlightCore/*.h` (Safety.h, Mission.h are new).
2. PIE: the Output Log should show `사전검증 PASS: 기준 임무 20/20 통과 (평균풍 0.0 m/s)` at start.
3. Optional: add a `Text_Safety` TextBlock to the HUD widget; fly past 50 m altitude -> expect HOLD, then LAND.
4. Check `Saved/FlightLogs/flight_*.csv` has the trailing `safety` column.
5. If it all works, remove the "v3의 UE 코드 ... 아직 빌드·실행하지 않았다" bullet in README 알려진 한계.
   If anything had to change, re-run the headless verify above (FlightCore changes must keep all 40 checks passing).

## Rules
- UE: no exceptions; no identifiers that clash with UE macros (check, verify, PI); no variable shadowing (error in UE5);
  keep existing component/UPROPERTY names (BP_Drone depends on them).
- New behavior needs a check in sim_main with pass criteria written before running it. Report failures honestly
  in the README dev log; never loosen a criterion silently.
- It is simulation with typical (not identified) parameters: never overclaim in docs.
- Public repo. Ask the user before committing or pushing.
