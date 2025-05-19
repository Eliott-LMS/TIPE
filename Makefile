# Nom de l'exécutable

TARGET = balle_graph.exe

# Compilateur
CC = x86_64-w64-mingw32-gcc

# Options de compilation
CFLAGS = -I/mnt/c/SDL2-2.32.2/x86_64-w64-mingw32/include -O4 -Wall

# Options de liaison
LDFLAGS = -L/mnt/c/SDL2-2.32.2/x86_64-w64-mingw32/lib -lmingw32 -lSDL2main -lSDL2_gfx -lSDL2_ttf -luser32 -lgdi32 -lwinmm  -lSDL2 -luuid -lrpcrt4 -lm

# Fichiers source
SRC = Simulation_Balle_18_05.c

# Dossier de destination
DEST = /mnt/d/_documents/TIPE
HERE = /root/Eliott/TIPE/Projet


# Règle par défaut
build: $(TARGET)
	if [ -f $(DEST)/$(TARGET) ]; then mv $(DEST)/$(TARGET) $(DEST)/$(TARGET).bak; fi
	cp $(TARGET) $(DEST)

all : # make build # make runm
	if [ -f $(DEST)/$(TARGET) ]; then mv $(DEST)/$(TARGET) $(DEST)/$(TARGET).bak; fi
	cp $(TARGET) $(DEST)
	$(DEST)/$(TARGET)


# Règle pour construire l'exécutable
$(TARGET): $(SRC)
	$(CC) -o $(TARGET) $(SRC) $(CFLAGS) $(LDFLAGS)

# Règle pour nettoyer les fichiers générés
clean:
	rm -f $(DEST)/$(TARGET)
	rm -f $(TARGET)

run : 
	$(DEST)/$(TARGET)

compare : 
	hyperline $(DEST)/$(TARGET)