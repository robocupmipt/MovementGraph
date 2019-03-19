CONFIG_NAME_FOR_TESTING		= linux-sdk
CONFIG_NAME_FOR_ROBOT		= atom-sdk
GEN_NAME			= generator.py
IP				= 192.168.1.2
VERTEXSET			= ./test/vertex.txt
EDGESET				= ./test/edge.txt
OUTPUT				= automaticInitGeneration.h
CMakeListsTest			= CMakeListsTest.txt
CMakeListsLib			= CMakeListsLib.txt
CMakeLists			= CMakeLists.txt

all: gen compil_for_testing compil_for_robot run

gen: $(GEN_NAME)
	python3 $(GEN_NAME) --vertex $(VERTEXSET) --edges $(EDGESET) --out $(OUTPUT)

compil_for_testing: $(CMakeListsTest)
	cp -f $(CMakeListsTest) $(CMakeLists)
	qibuild configure -c $(CONFIG_NAME_FOR_TESTING)
	qibuild make      -c $(CONFIG_NAME_FOR_TESTING)

compil_for_robot: $(CMakeListsLib)
	cp -f $(CMakeListsLib) $(CMakeLists)
	qibuild configure -c $(CONFIG_NAME_FOR_ROBOT)
	qibuild make      -c $(CONFIG_NAME_FOR_ROBOT)

make_test: gen compil_for_testing run

make_lib: gen compil_for_robot

run:
	./build-$(CONFIG_NAME_FOR_TESTING)/sdk/bin/graphcreator --pip ${IP}
