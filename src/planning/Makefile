include ../common.mk

CXXFLAGS = $(CXXFLAGS_STD) -O3 -DNDEBUG
LDFLAGS  = $(LDFLAGS_MAPPING) $(LDFLAGS_COMMON) $(LDFLAGS_STD)
LIBDEPS  = $(call libdeps, $(LDFLAGS))

LIB_PLANNING = $(LIB_PATH)/libplanning.a
LIBPLANNING_OBJS = \
	motion_planner.o \
	obstacle_distance_grid.o \
	astar.o

$(LIB_PLANNING): $(LIBPLANNING_OBJS) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $^

BIN_DIST_TEST  = $(BIN_PATH)/obstacle_distance_grid_test
BIN_ASTAR_TEST = $(BIN_PATH)/astar_test

ALL = $(LIB_PLANNING) $(BIN_DIST_TEST) $(BIN_ASTAR_TEST)

all: $(ALL)

$(BIN_DIST_TEST): obstacle_distance_grid_test.o $(LIBDEPS) $(LIB_PLANNING)
	@echo "\t$@"
	$(CXX) -o $@ $^ $(LIB_PLANNING) $(LDFLAGS) $(CXXFLAGS)

$(BIN_ASTAR_TEST): astar_test.o $(LIBDEPS) $(LIB_PLANNING)
	@echo "\t$@"
	$(CXX) -o $@ $^ $(LIB_PLANNING) $(LDFLAGS) $(CXXFLAGS)

	
clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
