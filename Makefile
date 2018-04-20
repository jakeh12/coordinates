all:
	gcc -o coord coord.c -lm -lpthread
clean:
	rm -f coord	
