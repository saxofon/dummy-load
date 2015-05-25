LDFLAGS += -lpthread -lrt

SRCS := $(wildcard *.c)

OBJS := $(SRCS:%.c=%.o)

APP := $(shell basename `pwd`)

all: $(APP)

$(APP): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) -o $@

clean:
	$(RM) $(APP) $(OBJS)
