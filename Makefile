all:
  g++ -std=c++17 -o astar *.cpp -lpthread -lm

clean:
  rm -f $(OBJS) $(OUT)
