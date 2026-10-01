# Nome da biblioteca
LIBRARY = libchipmunk.a

# Diretórios de origem e destino
SRCDIR = src
OBJDIR = obj
LIBDIR = lib

# Arquivos fonte e objeto
SOURCES = $(wildcard $(SRCDIR)/*.c)
OBJECTS = $(SOURCES:$(SRCDIR)/%.c=$(OBJDIR)/%.o)

# Flags de compilação
CC = gcc
CFLAGS = -Wall -Wextra -pedantic -std=gnu99  -ffast-math -O3 -DNDEBUG -I include
LDFLAGS = -O3 -DNDEBUG -lm

# Regra de compilação do objeto
$(OBJDIR)/%.o: $(SRCDIR)/%.c
	$(CC) -c -o $@ $< $(CFLAGS)

# Regra de compilação da biblioteca
$(LIBDIR)/$(LIBRARY): $(OBJECTS)
	ar rcs $@ $^

# Regra para limpar os arquivos objeto e biblioteca
clean:
	rm -f $(OBJDIR)/*.o 

# Regra padrão de compilação
all: $(LIBDIR)/$(LIBRARY)

$(shell mkdir -p $(OBJDIR))
$(shell mkdir -p $(LIBDIR))
.PHONY: all clean


