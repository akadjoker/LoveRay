# Makefile para compilar a Box2D em modo release

# Diretório de saída
OUT_DIR := out

# Flags de compilação
CXX := g++
CXXFLAGS := -std=c++11 -Wall -Wextra -Werror -pedantic -O2
LDFLAGS := -static-libgcc -static-libstdc++

# Arquivos fonte da Box2D
BOX2D_SRC := Box2D/Collision/*.cpp \
             Box2D/Common/*.cpp \
             Box2D/Dynamics/*.cpp \
             Box2D/Rope/*.cpp

# Arquivos objetos gerados
BOX2D_OBJ := $(patsubst %.cpp, $(OUT_DIR)/%.o, $(BOX2D_SRC))

# Nome da biblioteca gerada
LIB_NAME := libBox2D.a

# Regra padrão: compilação da biblioteca
all: $(OUT_DIR)/$(LIB_NAME)

# Regra para compilar os arquivos fonte da Box2D
$(OUT_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Regra para linkar os objetos e gerar a biblioteca
$(OUT_DIR)/$(LIB_NAME): $(BOX2D_OBJ)
	$(AR) rcs $@ $^

# Regra para limpar os arquivos gerados
clean:
	rm -rf $(OUT_DIR)

.PHONY: all clean

