
LIBPATH = libs

CXX = g++

CXXFLAGS =-I$(LIBPATH)/chipmunk/include/ -I$(LIBPATH)/box2d/include/box2d -I$(LIBPATH)/lua-5.5.5/src  -DPLATFORM_DESKTOP -std=c++11  -fsanitize=address -g #-fsanitize=undefined -fno-omit-frame-pointer -g
LIBS =-Llib -lraylib -llua -lbox2d -lchipmunk -lm


SRCDIR = src
OBJDIR = obj

SRCS = $(wildcard $(SRCDIR)/*.cpp)
OBJS = $(patsubst $(SRCDIR)/%.cpp,$(OBJDIR)/%.o,$(SRCS))


TARGET = love

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)
	./$(TARGET)

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(OBJDIR):
	mkdir -p $@

clean:
	rm -rf $(OBJDIR) $(TARGET)