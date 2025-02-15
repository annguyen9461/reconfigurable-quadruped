CC=g++
CXXFILES=i2c.cpp
OBJ=$(CXXFILES:.cpp=.o) $(CFILES:.c=.o)
TARGET=i2c

all: $(TARGET)

# imu: imu.o i2c.o
# 	$(CC) -o imu imu.o i2c.o

%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@

# %.o: %.c
# 	gcc -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)
