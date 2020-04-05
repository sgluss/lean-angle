test:
	g++ libraries/test/test.cpp -o output 
	chmod +x output
	./output

clean:
	rm output
