##############
#  Makefile  #
##############

CC = gcc
CFLAGS = -O2
OBJ = main.o Arcnet/arc_pci.o cp.o timer.o reverse.o command.o 
LIBS = -lm

all:robot

clean:
	/bin/rm -f *.o *~
.c.o.h:
	$(CC) $(CFLAGS) -c $(LIBS) $<

timer.o: timer.c timer.h main_cfg.h

Arcnet/arc_pci.o: Arcnet/arc_pci.h Arcnet/arc_pci_cfg.h main_cfg.h

main.o: main_cfg.h cfg_arm.h params_arm.h Arcnet/arc_pci.h timer.h reverse.h

cp.o: cp.c 


command.o: params_arm.h main_cfg.h

reverse.o: reverse.c


robot: $(OBJ) 
	$(CC) -o robot $(OBJ) $(LIBS)


