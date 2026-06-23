# auv_drivers

Sim-to-real seam (spec §9.3). **v1 contains stubs/interfaces only.**

`driver_contract.py` defines the exact topic/type/frame each real driver must publish.
Because real drivers publish the **same** interface-contract topics the simulators do,
bringing up hardware is "launch the driver instead of the sim adapter" — no SLAM-code
changes. `driver_stub` logs the contract for a given `driver:=` and idles; replace its
body with a real device driver (Water Linked / Nortek DVL, AHRS, sonar, pressure).
