include ../common.mk

# flags for building the gtk library
CXXFLAGS = $(CXXFLAGS_STD) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_IMAGESOURCE) \
	$(CFLAGS_MATH) \
	$(CFLAGS_COMMON) \
	$(CFLAGS_LCMTYPES) \
	$(CFLAGS_GLIB) \
	$(CFLAGS_EECS467) \
	#-O2
	
CFLAGS = $(CFLAGS_STD) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_IMAGESOURCE) \
	$(CFLAGS_MATH) \
	$(CFLAGS_COMMON) \
	$(CFLAGS_LCMTYPES) \
	$(CFLAGS_GLIB) \
	$(CFLAGS_EECS467) \
	#-O2

LDFLAGS = $(LDFLAGS_STD) \
	$(LDFLAGS_VX_GTK) \
	$(LDFLAGS_IMAGESOURCE) \
	$(LDFLAGS_MATH) \
	$(LDFLAGS_COMMON) \
	$(LDFLAGS_LCMTYPES) \
	$(LDFLAGS_PLANNING) \
	$(LDFLAGS_EECS467)

LIBDEPS = $(call libdeps, $(LDFLAGS))

BIN_ARM_CONTROLLER 	= $(BIN_PATH)/arm_controller
BIN_LAUNDRYMAN	= $(BIN_PATH)/laundryman

ALL = $(BIN_ARM_CONTROLLER) $(BIN_LAUNDRYMAN)


all: $(ALL)

$(BIN_ARM_CONTROLLER): ArmController.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)
	
$(BIN_LAUNDRYMAN): laundry.o ImageManager.o VxGui.o blob.o colorspace.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)

