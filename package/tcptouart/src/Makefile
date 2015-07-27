tcptouart: main.o
	$(CC) $(LDFLAGS) main.o -o tcptouart -lpthread
main.o: main.c
	$(CC) $(CFLAGS) -c main.c -I mavlink/include/mavlink/v1.0
clean:
	rm *.o tcptouart
