build-test:
	g++ -g ./src/lane_detector_test.cpp ./src/lane_detector.cpp -I./include/ \
	    `pkg-config --cflags --libs opencv` \
	    -fsanitize=address,undefined -fno-omit-frame-pointer
