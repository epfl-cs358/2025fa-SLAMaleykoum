# ----------------------------
#       Compiler
# ----------------------------
CXX = clang++
CXXFLAGS = -std=c++17 -I include -I src -I/opt/homebrew/include
LDFLAGS = -L/opt/homebrew/lib
SFML_LIBS = -lsfml-graphics -lsfml-window -lsfml-system -pthread

# ----------------------------
#       Sources communes
# ----------------------------
BAYESIAN_SRC = src/esp1/mapping/occupancy/bayesian_grid.cpp
PLANNER_SRC  = src/esp1/planning/global_planner.cpp
MISSION_SRC  = src/esp1/planning/mission_planner.cpp

# ----------------------------
#       Tests
# ----------------------------
STATIC_TEST_SRC   = src/esp1/test/test_bayesian_grid_static.cpp
DYNAMIC_TEST_SRC  = src/esp1/test/test_bayesian_grid_dynamic.cpp
PLANNER_TEST_SRC  = src/esp1/test/test_global_planner.cpp
MISSION_TEST_SRC  = src/esp1/test/test_mission_planner.cpp

# ----------------------------
#       Executables
# ----------------------------
STATIC_TEST_BIN   = test_bayesian_grid_static
DYNAMIC_TEST_BIN  = test_bayesian_grid_dynamic
PLANNER_TEST_BIN  = test_global_planner
MISSION_TEST_BIN  = test_mission_planner

# ----------------------------
#       Default target
# ----------------------------
all: $(STATIC_TEST_BIN) $(DYNAMIC_TEST_BIN) $(PLANNER_TEST_BIN) $(MISSION_TEST_BIN)

# ----------------------------
#       Bayesian Grid Static
# ----------------------------
$(STATIC_TEST_BIN): $(STATIC_TEST_SRC) $(BAYESIAN_SRC)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) $(SFML_LIBS)

# ----------------------------
#       Bayesian Grid Dynamic
# ----------------------------
$(DYNAMIC_TEST_BIN): $(DYNAMIC_TEST_SRC) $(BAYESIAN_SRC)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) $(SFML_LIBS)

# ----------------------------
#       Global Planner Test
# ----------------------------
$(PLANNER_TEST_BIN): $(PLANNER_TEST_SRC) $(BAYESIAN_SRC) $(PLANNER_SRC)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) $(SFML_LIBS)

# ----------------------------
#       Mission Planner Frontier Test
# ----------------------------
$(MISSION_TEST_BIN): $(MISSION_TEST_SRC) $(BAYESIAN_SRC) $(MISSION_SRC)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) $(SFML_LIBS)

# ----------------------------
#       Clean
# ----------------------------
clean:
	rm -f $(STATIC_TEST_BIN) $(DYNAMIC_TEST_BIN) $(PLANNER_TEST_BIN) $(MISSION_TEST_BIN)

.PHONY: all clean
