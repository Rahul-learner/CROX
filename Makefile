.PHONY: all drone gs clean flash-drone flash-gs monitor plot

all: drone gs

drone:
	cmake --preset drone
	cmake --build --preset drone -j4

gs:
	cd GroundStation && cmake --preset groundstation
	cd GroundStation && cmake --build --preset groundstation -j4

clean:
	rm -rf build_drone
	rm -rf GroundStation/build_gs

flash-drone: drone
	picotool load -xf build_drone/CROX.uf2

flash-gs: gs
	picotool load -xf GroundStation/build_gs/GroundStation.uf2

monitor:
	cd configurator && cargo run --release

plot:
	./scripts/.venv/bin/python scripts/read_blackbox_and_plot.py