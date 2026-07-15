.PHONY: drone gs clean

drone:
	cmake --preset drone
	cmake --build --preset drone -j4

gs:
	cd GroundStation && cmake --preset groundstation
	cd GroundStation && cmake --build --preset groundstation -j4

clean:
	rm -rf build_drone
	rm -rf GroundStation/build_gs
