++ := g++
CXXFLAGS=-g -std=c++11 -Wall -pedantic
exe := play
obj := Maze.o Search.o DFS.o BFS.o Greedy.o AStar.o

maze: $(obj)
	$(++) -o $(exe) $(obj)

%o: %c
	$(++) -c $^ -o $@

#Maze.o: Maze.cpp
#	$(++) -c maze.cpp

#Search.o: Search.cpp Search.h
#	$(++) -c Search.cpp

#DFS.o: DFS.cpp Search.h
#	$(++) -c DFS.cpp

.PHONY:clean
clean:
	rm -rf $(obj) $(exe)
