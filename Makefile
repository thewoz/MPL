INCLUDE_INSTALL_PATH=/usr/local/include
LIBRARY_INSTALL_PATH=/usr/local/lib

LIBRARY_NAME=mpl


all: install


install:
	@rm -rf $(INCLUDE_INSTALL_PATH)/$(LIBRARY_NAME)
	@ln -s $(shell pwd)/include $(INCLUDE_INSTALL_PATH)/$(LIBRARY_NAME)

uninstall:
	@rm -rf $(INCLUDE_INSTALL_PATH)/$(LIBRARY_NAME)

test:
	@mkdir ./bin
	g++ -march=native -Os -std=c++1z -I./include ./src/lagrange.cpp -o ./bin/lagrange
	g++ -march=native -Os -std=c++1z -I/usr/local/include -I./include ./src/curl.cpp -L/usr/local/lib -lcurl -o ./bin/curl

