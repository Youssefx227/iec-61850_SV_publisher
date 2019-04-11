
	
	
	
CC=gcc
CFLAGS=-Wall -O -lpthread -lm
LDFLAGS=-static
LIBPATH = -L../libiec61850-1.3.3/build/
LIB = -liec61850
EXEC=sv_publisher
INCLUDE = -I../inc
LIBIEC_HOME = /home/nathan/Documents/youssef_C/libiec61850-1.3.3/

PROJECT_SOURCES = sv_publisher_example.c
PROJECT_SOURCES += static_model.c

PROJECT_ICD_FILE = sv.icd

include /home/nathan/Documents/youssef_C/libiec61850-1.3.3/make/target_system.mk
include /home/nathan/Documents/youssef_C/libiec61850-1.3.3/make/stack_includes.mk

all: $(EXEC)

include $(LIBIEC_HOME)/make/common_targets.mk

model:	$(PROJECT_ICD_FILE)
	java -jar $(LIBIEC_HOME)/tools/model_generator/genmodel.jar $(PROJECT_ICD_FILE)

$(EXEC): $(PROJECT_SOURCES)
				$(CC) $(INCLUDE) $(PROJECT_SOURCES)  -o $(EXEC) $(CFLAGS) $(LIBPATH) $(LIB) 

clean:
	rm *.o