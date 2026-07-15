.PHONY: all drone gs gs-monitor clean flash-drone flash-gs monitor plot

all: drone gs gs-monitor

drone:
	cmake --preset drone
	cmake --build --preset drone -j4

gs:
	cd GroundStation && cmake --preset groundstation
	cd GroundStation && cmake --build --preset groundstation -j4

gs-monitor:
	cd GroundStation/GS_Monitor && cmake --preset gs_monitor
	cd GroundStation/GS_Monitor && cmake --build --preset gs_monitor -j4

clean:
	rm -rf build_drone
	rm -rf GroundStation/build_gs
	rm -rf GroundStation/GS_Monitor/build_gs_monitor

flash-drone: drone
	picotool load -xf build_drone/CROX.uf2

flash-gs: gs
	picotool load -xf GroundStation/build_gs/GroundStation.uf2

monitor:
	./scripts/.venv/bin/python scripts/debug.py --port /dev/ttyACM0 --baud 115200

plot:
	./scripts/.venv/bin/python scripts/read_blackbox_and_plot.py

monitor-gs:
	./GroundStation/GS_Monitor/build_gs_monitor/GS_Monitor