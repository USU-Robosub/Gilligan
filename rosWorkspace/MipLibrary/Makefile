CC=g++
CXXFLAGS=
AR=ar
SOURCE=byteswap_utilities.c mip_sdk_3dm.c mip_sdk_ahrs.c mip_sdk_base.c mip_sdk_gps.c mip_sdk_interface.c mip_sdk_nav.c mip_sdk_system.c mip_sdk_user_functions.c mip.c ring_buffer.c
TARGET=libMipLibrary.a
TARGET_DIR=lib
LIB_OBJ=$(addprefix build/, $(SOURCE:.c=.o))
OUT_DIRS=build lib

all: directories $(TARGET_DIR)/$(TARGET)

$(TARGET_DIR)/$(TARGET): $(LIB_OBJ)
	$(AR) rcs $(TARGET_DIR)/$(TARGET) $(LIB_OBJ)

directories:
	mkdir -p $(OUT_DIRS)

build/%.o: src/%.c
	$(CC) -c $< $(CXXFLAGS) -o $@

clean:
	rm -rf $(OUT_DIRS)